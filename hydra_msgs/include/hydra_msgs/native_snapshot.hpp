// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

#pragma once
// Local, versioned visualization snapshot. Atomic rename keeps readers on a complete
// inode. The graph and indexed mesh share one header, so dropped ROS notifications
// cannot pair geometry with a different graph update. Native-endian, same-host only.
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace hydra_msgs::native_snapshot {
constexpr uint64_t kMagic = 0x4859445241445347ULL;
struct Header {
  uint64_t magic = kMagic, version = 1, sequence = 0, timestamp = 0;
  uint64_t vertices = 0, faces = 0, graph_bytes = 0, frame_bytes = 0;
};
struct Vertex { float x, y, z; uint8_t r, g, b, a; };
using Face = std::array<uint64_t, 3>;
static_assert(sizeof(Header) == 64 && sizeof(Vertex) == 16 && sizeof(Face) == 24);
inline std::string defaultPath() {
  const char* mesh = std::getenv("HYDRA_NATIVE_MESH_PATH");
  return (std::filesystem::path(mesh ? mesh : "/dev/shm/hydra-native/mesh.bin")
              .parent_path() / "dsg_snapshot.bin").string();
}
class Mapping {
 public:
  Mapping(const std::string& path, size_t bytes = 0) {
    const bool write = bytes != 0;
    int fd = ::open(path.c_str(), write ? O_RDWR | O_CREAT | O_TRUNC : O_RDONLY, 0640);
    if (fd < 0) throw std::runtime_error("snapshot open: " + std::string(strerror(errno)));
    if (write) {
      const int error = ::posix_fallocate(fd, 0, bytes);
      if (error) { ::close(fd); throw std::runtime_error("snapshot allocation: " + std::string(strerror(error))); }
    }
    struct stat status{};
    if (::fstat(fd, &status) || status.st_size < static_cast<off_t>(sizeof(Header))) {
      ::close(fd); throw std::runtime_error("invalid snapshot size");
    }
    size_ = status.st_size;
    data_ = ::mmap(nullptr, size_, write ? PROT_READ | PROT_WRITE : PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (data_ == MAP_FAILED) throw std::runtime_error("snapshot mmap failed");
  }
  ~Mapping() { if (data_ != MAP_FAILED) ::munmap(data_, size_); }
  Mapping(const Mapping&) = delete;
  Mapping& operator=(const Mapping&) = delete;
  uint8_t* data() const { return static_cast<uint8_t*>(data_); }
  size_t size() const { return size_; }
 private:
  void* data_ = MAP_FAILED;
  size_t size_ = 0;
};
inline size_t advance(size_t offset, uint64_t count, size_t stride, size_t limit) {
  if (offset > limit || count > (limit - offset) / stride)
    throw std::runtime_error("snapshot length overflow or truncation");
  return offset + count * stride;
}
class Writer {
 public:
  explicit Writer(std::string path) : path_(std::move(path)) {
    std::filesystem::create_directories(std::filesystem::path(path_).parent_path());
    lock_ = ::open((path_ + ".lock").c_str(), O_CREAT | O_RDWR, 0640);
    if (lock_ < 0 || ::flock(lock_, LOCK_EX | LOCK_NB)) {
      if (lock_ >= 0) ::close(lock_);
      throw std::runtime_error("another snapshot writer owns " + path_);
    }
    ::unlink(path_.c_str());
  }
  ~Writer() { ::unlink(path_.c_str()); ::close(lock_); }
  Writer(const Writer&) = delete;
  Writer& operator=(const Writer&) = delete;
  template <typename GetVertex, typename GetFace>
  void write(Header header, const std::string& frame, const std::vector<uint8_t>& graph,
             GetVertex vertex, GetFace face) {
    header.frame_bytes = frame.size(); header.graph_bytes = graph.size();
    const size_t limit = std::numeric_limits<off_t>::max();
    size_t bytes = advance(sizeof(Header), frame.size(), 1, limit);
    bytes = advance(bytes, graph.size(), 1, limit);
    bytes = advance(bytes, header.vertices, sizeof(Vertex), limit);
    bytes = advance(bytes, header.faces, sizeof(Face), limit);
    const auto temp = path_ + ".tmp";
    try {
      Mapping map(temp, bytes);
      auto* out = map.data();
      std::memcpy(out, &header, sizeof(header)); out += sizeof(header);
      std::memcpy(out, frame.data(), frame.size()); out += frame.size();
      if (!graph.empty()) std::memcpy(out, graph.data(), graph.size());
      out += graph.size();
      for (uint64_t i = 0; i < header.vertices; ++i) {
        const Vertex v = vertex(i); std::memcpy(out, &v, sizeof(v)); out += sizeof(v);
      }
      for (uint64_t i = 0; i < header.faces; ++i) {
        const Face f = face(i); std::memcpy(out, &f, sizeof(f)); out += sizeof(f);
      }
      if (::rename(temp.c_str(), path_.c_str())) throw std::runtime_error("snapshot rename failed");
    } catch (...) { ::unlink(temp.c_str()); throw; }
  }
 private:
  std::string path_;
  int lock_ = -1;
};
class Reader {
 public:
  explicit Reader(const std::string& path) : map_(path) {
    std::memcpy(&header, map_.data(), sizeof(header));
    if (header.magic != kMagic || header.version != 1) throw std::runtime_error("unsupported snapshot format");
    const auto graph_offset = advance(sizeof(Header), header.frame_bytes, 1, map_.size());
    vertices_ = advance(graph_offset, header.graph_bytes, 1, map_.size());
    faces_ = advance(vertices_, header.vertices, sizeof(Vertex), map_.size());
    if (advance(faces_, header.faces, sizeof(Face), map_.size()) != map_.size())
      throw std::runtime_error("snapshot has trailing bytes");
    frame.assign(reinterpret_cast<const char*>(map_.data() + sizeof(Header)), header.frame_bytes);
    graph.assign(map_.data() + graph_offset, map_.data() + vertices_);
  }
  Vertex vertex(uint64_t i) const {
    if (i >= header.vertices) throw std::out_of_range("snapshot vertex");
    Vertex value; std::memcpy(&value, map_.data() + vertices_ + i * sizeof(value), sizeof(value)); return value;
  }
  Face face(uint64_t i) const {
    if (i >= header.faces) throw std::out_of_range("snapshot face");
    Face value; std::memcpy(&value, map_.data() + faces_ + i * sizeof(value), sizeof(value)); return value;
  }
  Header header;
  std::string frame;
  std::vector<uint8_t> graph;
 private:
  Mapping map_;
  size_t vertices_, faces_;
};
}  // namespace hydra_msgs::native_snapshot
