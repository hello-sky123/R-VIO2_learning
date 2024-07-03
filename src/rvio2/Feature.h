/**
 * This file is part of R-VIO2.
 *
 * Copyright (C) 2022 Zheng Huai <zhuai@udel.edu> and Guoquan Huang
 * <ghuang@udel.edu> For more information see <http://github.com/rpng/R-VIO2>
 *
 * R-VIO2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * R-VIO2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with R-VIO2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef NODE_H
#define NODE_H

#include <Eigen/Core>

namespace RVIO2 {

class Feature {
 public:
  // 常量限定只在函数定义时有效
  Feature(int nFeatureId, int nImageId);

  // 完全在class内部定义的函数，自动成为inline函数
  void Inited() { mbIsInited = true; }

  void Marginalized() { mbIsMarginalized = true; }

  void SetPosition(const Eigen::Vector3f& position) {
    mPosition = position;
  }

  void SetFejPosition(const Eigen::Vector3f& position) {
    mFejPosition = position;
  }

  // 使用const来修饰基本类型是多余的，因为它们的值本身就不能被更改
  int FeatureId() const { return mnFeatureId; }

  int RootImageId() const { return mnRootImageId; }

  bool IsInited() const { return mbIsInited; }

  bool IsMarginalized() const { return mbIsMarginalized; }

  Eigen::Vector3f& Position() { return mPosition; }

  Eigen::Vector3f& FejPosition() { return mFejPosition; }

  void reset(const int nImageId) {
    mnRootImageId = nImageId;
    mbIsInited = false;
    mbIsMarginalized = false;
  }

  void clear() {
    mnRootImageId = -1;
    mbIsInited = false;
    mbIsMarginalized = false;
  }

 private:
  int mnFeatureId;    // start from 0
  int mnRootImageId;  // start from 0

  bool mbIsInited;
  bool mbIsMarginalized;

  Eigen::Vector3f mPosition;
  Eigen::Vector3f mFejPosition;
};

}  // namespace RVIO2

#endif
