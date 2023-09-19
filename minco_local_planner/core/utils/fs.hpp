/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-04 15:24:02
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2023-09-05 00:07:10
 */
#include <stdint.h>

#ifndef __FS_HPP__
#define __FS_HPP__

#include <filesystem>
#include <string>
namespace minco_local_planner::utils::fs {

inline bool IsFileExist(const std::string &filename) {
  return std::filesystem::exists(filename);
}

inline std::string Dirname(const std::string &full_path) {
  auto start_pos = full_path.find_first_not_of("\t ");  //! 去除左边的空白符
  auto end_pos = full_path.find_last_of('/');

  //! 如果全是空白符或没有找到/，则推测为当前目录
  if (start_pos == std::string::npos || end_pos == std::string::npos)
    return ".";

  //! 对以/开头的要特征处理，否则 "/a" 就会被处理成 ""
  if (start_pos == end_pos) return "/";

  return full_path.substr(start_pos, end_pos - start_pos);
}
}  // namespace minco_local_planner::utils::fs

#endif /* __FS_HPP__ */
