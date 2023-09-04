/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-04 15:13:52
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-04 20:36:21
 */
#include <stdint.h>

#ifndef __JSON_DEEP_LOADER_HPP__
#define __JSON_DEEP_LOADER_HPP__

#include "fs.hpp"
#include "json.hpp"
#include "string.hpp"
namespace minco_local_planner::utils {
const std::string kInclude = "__include__";

//! include描述类型不合法，不是string
struct IncludeDescriptorTypeInvalid : public std::runtime_error {
  explicit IncludeDescriptorTypeInvalid()
      : std::runtime_error(
            "include descriptor type error, it should be string") {}
};
//! 重复include同一个文件
struct DuplicateIncludeError : public std::runtime_error {
  explicit DuplicateIncludeError(const std::string &include_file)
      : std::runtime_error("duplicate include file:" + include_file) {}
};

class DeepLoader {
 public:
  Json load(const std::string &filename) {
    if (checkDuplicateInclude(filename)) throw DuplicateIncludeError(filename);

    files_.push_back(filename);
    Json js = Load(filename);
    traverse(js);
    files_.pop_back();
    return js;
  }

 protected:
  void traverse(Json &js) {
    if (js.is_object()) {
      Json js_patch;
      for (auto &item : js.items()) {
        auto &js_value = item.value();
        if (item.key() == kInclude) {
          handleInclude(js_value, js_patch);
        } else {
          traverse(js_value);
        }
      }
      js.erase(kInclude);

      if (!js_patch.is_null()) js.merge_patch(js_patch);

    } else if (js.is_array()) {
      for (auto &js_item : js) {
        traverse(js_item);
      }
    }
  }
  void handleInclude(const Json &js_include, Json &js_parent) {
    if (js_include.is_string()) {
      includeByDescriptor(js_include.get<std::string>(), js_parent);
    } else if (js_include.is_array()) {
      for (auto &js_item : js_include) {
        if (js_item.is_string())
          includeByDescriptor(js_item.get<std::string>(), js_parent);
        else
          throw IncludeDescriptorTypeInvalid();
      }
    } else {
      throw IncludeDescriptorTypeInvalid();
    }
  }
  void includeByDescriptor(const std::string &descriptor, Json &js) {
    std::vector<std::string> str_vec;
    string::Split(descriptor, "=>", str_vec);
    std::string filename = string::Strip(str_vec.at(0));

    std::string real_filename;
    if (filename.front() != '/') {
      auto dirname = fs::Dirname(files_.back());
      real_filename = dirname + '/' + filename;
    } else {
      real_filename = filename;
    }

    auto js_load = load(real_filename);

    if (str_vec.size() >= 2) {
      auto keyname = string::Strip(str_vec.at(1));
      js[keyname] = std::move(js_load);
    } else {
      js.merge_patch(std::move(js_load));
    }
  }
  bool checkDuplicateInclude(const std::string &filename) const {
    return std::find(files_.begin(), files_.end(), filename) != files_.end();
  }

 private:
  std::vector<std::string> files_;
};

inline Json LoadDeeply(const std::string &filename) {
  return DeepLoader().load(filename);
}

}  // namespace minco_local_planner::utils
#endif /* __JSON_DEEP_LOADER_HPP__ */
