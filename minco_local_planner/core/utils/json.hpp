/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-04 14:55:40
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-04 16:43:05
 */
#include <stdint.h>

#ifndef __JSON_HPP__
#define __JSON_HPP__
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "3rd-party/nlohmann/json.hpp"
#include "3rd-party/nlohmann/json_fwd.hpp"
namespace minco_local_planner::utils {
using Json = nlohmann::json;
using OrderedJson = nlohmann::ordered_json;

inline bool Get(const Json &js, Json &field_value) {
  if (!js.is_object()) return false;
  field_value = js.get<Json>();
  return true;
}

//! true, false
inline bool Get(const Json &js, bool &field_value) {
  if (!js.is_boolean()) return false;
  field_value = js.get<bool>();
  return true;
}

inline bool Get(const Json &js, unsigned int &field_value) {
  if (!js.is_number_unsigned()) return false;
  field_value = js.get<unsigned int>();
  return true;
}

inline bool Get(const Json &js, double &field_value) {
  if (!js.is_number()) return false;
  field_value = js.get<double>();
  return true;
}

inline bool Get(const Json &js, std::string &field_value) {
  if (!js.is_string()) return false;
  field_value = js.get<std::string>();
  return true;
}

inline bool Get(const Json &js, int &field_value) {
  if (!js.is_number_integer()) return false;
  field_value = js.get<int>();
  return true;
}

inline bool GetField(const Json &js, const std::string &field_name,
                     Json &field_value) {
  if (!js.contains(field_name)) return false;
  return Get(js.at(field_name), field_value);
}

inline bool GetField(const Json &js, const std::string &field_name,
                     bool &field_value) {
  if (!js.contains(field_name)) return false;
  return Get(js.at(field_name), field_value);
}

inline bool GetField(const Json &js, const std::string &field_name,
                     unsigned int &field_value) {
  if (!js.contains(field_name)) return false;
  return Get(js.at(field_name), field_value);
}

inline bool GetField(const Json &js, const std::string &field_name,
                     int &field_value) {
  if (!js.contains(field_name)) return false;
  return Get(js.at(field_name), field_value);
}

inline bool GetField(const Json &js, const std::string &field_name,
                     double &field_value) {
  if (!js.contains(field_name)) return false;
  return Get(js.at(field_name), field_value);
}

inline bool GetField(const Json &js, const std::string &field_name,
                     std::string &field_value) {
  if (!js.contains(field_name)) return false;
  return Get(js.at(field_name), field_value);
}

inline bool HasObjectField(const Json &js, const std::string &field_name) {
  if (!js.contains(field_name)) return false;
  auto &js_field = js.at(field_name);
  return js_field.is_object();
}

inline bool HasArrayField(const Json &js, const std::string &field_name) {
  if (!js.contains(field_name)) return false;
  auto &js_field = js.at(field_name);
  return js_field.is_array();
}

inline bool HasBooleanField(const Json &js, const std::string &field_name) {
  if (!js.contains(field_name)) return false;
  auto &js_field = js.at(field_name);
  return js_field.is_boolean();
}

inline bool HasNumberField(const Json &js, const std::string &field_name) {
  if (!js.contains(field_name)) return false;
  auto &js_field = js.at(field_name);
  return js_field.is_number();
}

inline bool HasFloatField(const Json &js, const std::string &field_name) {
  if (!js.contains(field_name)) return false;
  auto &js_field = js.at(field_name);
  return js_field.is_number_float();
}

inline bool HasIntegerField(const Json &js, const std::string &field_name) {
  if (!js.contains(field_name)) return false;
  auto &js_field = js.at(field_name);
  return js_field.is_number_integer();
}

inline bool HasUnsignedField(const Json &js, const std::string &field_name) {
  if (!js.contains(field_name)) return false;
  auto &js_field = js.at(field_name);
  return js_field.is_number_unsigned();
}
//! 打开文件失败异常
struct OpenFileError : public std::runtime_error {
  explicit OpenFileError(const std::string &filename)
      : std::runtime_error("open file " + filename + " fail") {}
};
//! 解析JSON文件失败异常
struct ParseJsonFileError : public std::runtime_error {
  explicit ParseJsonFileError(const std::string &filename,
                              const std::string &detail)
      : std::runtime_error("parse json file " + filename +
                           " fail, detail:" + detail) {}
};

/// 加载JSON文件
/**
 * \param filename  JSON文件名
 * \return Json     解析所得的Json对象
 *
 * \throw OpenFileError
 *        ParseJsonFileError
 */
inline Json Load(const std::string &filename) {
  Json js;
  std::ifstream input_json_file(filename);
  if (input_json_file.is_open()) {
    try {
      input_json_file >> js;
    } catch (const std::exception &e) {
      throw ParseJsonFileError(filename, e.what());
    }
  } else {
    throw OpenFileError(filename);
  }

  return js;
}
}  // namespace minco_local_planner::utils
#endif /* __JSON_HPP__ */
