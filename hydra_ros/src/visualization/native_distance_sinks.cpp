// Copyright (c) 2026, IHMC Robotics Lab.
// All rights reserved.

// Native RDX views of Hydra's active TSDF and GVD distance field.
#include <hydra/reconstruction/reconstruction_module.h>
#include <hydra/frontend/gvd_place_extractor.h>
#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>

namespace hydra {
struct NativeDistanceConfig {
  double rate_hz = 2.0;
  float min_weight = 0.0001f;
};
void declare_config(NativeDistanceConfig& c) {
  config::name("NativeDistanceConfig");
  config::field(c.rate_hz, "rate_hz");
  config::field(c.min_weight, "min_weight");
  config::check(c.rate_hz, config::GT, 0.0, "rate_hz");
}

class NativeDistanceWriter {
 public:
  NativeDistanceWriter(const char* name, uint64_t primitive, double rate)
      : primitive_(primitive), interval_(1.0 / rate) {
    const char* mesh = std::getenv("HYDRA_NATIVE_MESH_PATH");
    path_ = (std::filesystem::path(mesh ? mesh : "/opt/hydra_mesh/mesh.bin").parent_path() / name).string();
    std::filesystem::create_directories(std::filesystem::path(path_).parent_path());
    lock_ = open((path_ + ".lock").c_str(), O_CREAT | O_RDWR, 0640);
    if (lock_ < 0 || flock(lock_, LOCK_EX | LOCK_NB)) {
      if (lock_ >= 0) close(lock_);
      throw std::runtime_error("Native distance output is locked: " + path_);
    }
    sequence_ = std::chrono::system_clock::now().time_since_epoch().count();
    unlink((path_ + ".height").c_str());
    write({}, 0);
  }
  ~NativeDistanceWriter() {
    unlink(path_.c_str()); unlink((path_ + ".tmp").c_str());
    unlink((path_ + ".height").c_str()); close(lock_);
  }
  bool ready() {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_ < interval_) return false;
    last_ = now;
    return true;
  }
  float height() const {
    float h = 1.0f;
    std::ifstream input(path_ + ".height");
    float requested;
    if ((input >> requested) && std::isfinite(requested) && requested >= -10 && requested <= 20) h = requested;
    return h;
  }
  void write(const std::vector<float>& vertices, uint64_t stamp) {
    const uint64_t bytes = 64 + vertices.size() * sizeof(float);
    if (bytes > 1500000000ULL) { LOG(ERROR) << "Distance snapshot too large"; return; }
    const auto tmp = path_ + ".tmp";
    int fd = open(tmp.c_str(), O_CREAT | O_TRUNC | O_RDWR, 0640);
    if (fd < 0) { LOG(ERROR) << "Cannot open " << tmp; return; }
    int error = posix_fallocate(fd, 0, bytes);
    if (error) { close(fd); unlink(tmp.c_str()); LOG(ERROR) << strerror(error); return; }
    void* mapped = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mapped == MAP_FAILED) { close(fd); unlink(tmp.c_str()); LOG(ERROR) << "Cannot map " << tmp; return; }
    const uint64_t header[8] = {0x48594452414d4553ULL, 1, ++sequence_, vertices.size()/7, bytes, stamp, 28, primitive_};
    std::memcpy(mapped, header, 64);
    if (!vertices.empty()) std::memcpy(static_cast<char*>(mapped) + 64, vertices.data(), bytes - 64);
    munmap(mapped, bytes); close(fd);
    if (rename(tmp.c_str(), path_.c_str())) { unlink(tmp.c_str()); LOG(ERROR) << "Cannot publish " << path_; }
  }
 private:
  std::string path_;
  int lock_;
  uint64_t primitive_, sequence_;
  std::chrono::duration<double> interval_;
  std::chrono::steady_clock::time_point last_{};
};

class NativeTsdfSink : public ReconstructionModule::Sink {
 public:
  explicit NativeTsdfSink(const NativeDistanceConfig& config)
      : config_(config), writer_("tsdf.bin", 1, config.rate_hz), active_mesh_writer_("active_mesh.bin", 0, config.rate_hz) {}
  void call(uint64_t stamp, const Eigen::Isometry3d&, const TsdfLayer& layer,
            const ReconstructionOutput& output) override {
    if (!writer_.ready()) return;
    // Export the selected backend's existing active mesh, never the persistent DSG archive.
    // output.map() is cloned before archival; explicitly omit the blocks being archived.
    const BlockIndexSet archived(output.archived_blocks.begin(), output.archived_blocks.end());
    std::vector<float> mesh_vertices;
    for (const auto& block : output.map().getMeshLayer()) {
      if (archived.count(block.index)) continue;
      for (const auto& face : block.faces) {
        if (face[0] >= block.points.size() || face[1] >= block.points.size() || face[2] >= block.points.size()) continue;
        if (!block.points[face[0]].allFinite() || !block.points[face[1]].allFinite() || !block.points[face[2]].allFinite()) continue;
        for (const auto index : face) {
          const auto& p = block.points[index];
          mesh_vertices.insert(mesh_vertices.end(), {p.x(), p.y(), p.z(), .5f, .5f, .5f, 1.f});
        }
      }
    }
    active_mesh_writer_.write(mesh_vertices, stamp);
    const float band = std::max(output.map().config.truncation_distance, 1e-6f);
    std::vector<float> points;
    for (const auto& block : layer) {
      for (size_t i = 0; i < block.numVoxels(); ++i) {
        const auto& v = block.getVoxel(i);
        if (v.weight < config_.min_weight || !std::isfinite(v.distance) || std::abs(v.distance) >= band) continue;
        const auto p = block.getVoxelPosition(i);
        const float d = std::clamp(v.distance / band, -1.0f, 1.0f);
        points.insert(points.end(), {p.x(), p.y(), p.z(), std::clamp(2*d, 0.f, 1.f),
          std::clamp(2-2*std::abs(d), 0.f, 1.f), std::clamp(-2*d, 0.f, 1.f), 1.f});
      }
    }
    writer_.write(points, stamp);
  }
 private:
  NativeDistanceConfig config_;
  NativeDistanceWriter writer_;
  NativeDistanceWriter active_mesh_writer_;
};

class NativeGvdDistanceSink : public GvdPlaceExtractor::Sink {
 public:
  explicit NativeGvdDistanceSink(const NativeDistanceConfig& config)
      : writer_("esdf.bin", 2, config.rate_hz) {}
  void call(uint64_t stamp, const Eigen::Isometry3f&, const places::GvdLayer& layer,
            const places::GraphExtractorInterface*) override {
    if (!writer_.ready()) return;
    const float height = writer_.height();
    const float size = layer.voxel_size;
    const auto z_index = static_cast<long>(std::floor(height / size));
    std::vector<float> cells;
    for (const auto& block : layer) {
      // Reject whole blocks outside the requested slab before scanning voxels.
      const float lower_z = block.index.z() * size * layer.voxels_per_side;
      if (height < lower_z || height >= lower_z + size * layer.voxels_per_side) continue;
      for (size_t i = 0; i < block.numVoxels(); ++i) {
        const auto& v = block.getVoxel(i);
        if (!v.observed || !std::isfinite(v.distance)) continue;
        const auto p = block.getVoxelPosition(i);
        if (static_cast<long>(std::floor(p.z() / size)) != z_index) continue;
        cells.insert(cells.end(), {p.x()-size/2, p.y()-size/2, height, v.distance, size, 0.f, 0.f});
      }
    }
    writer_.write(cells, stamp);
  }
 private:
  NativeDistanceWriter writer_;
};
static const auto tsdf_registration = config::RegistrationWithConfig<ReconstructionModule::Sink, NativeTsdfSink, NativeDistanceConfig>("NativeTsdfSink");
static const auto gvd_registration = config::RegistrationWithConfig<GvdPlaceExtractor::Sink, NativeGvdDistanceSink, NativeDistanceConfig>("NativeGvdDistanceSink");
}  // namespace hydra
