//------------------------------------------------------------------------------
/// @file
/// @brief    Pos クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCAction.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// @brief 整数値における二次元の値を表します。
    ///
    /// 二次元の要素を表すための型です。
    class Pos
    {
    public:
        Pos();
        Pos(int x, int y);

        bool operator==(const Pos& aRhs) const; ///< 等しい
        bool operator!=(const Pos& aRhs) const; ///< 等しくない

        const Pos move(Action aAction) const; ///< 指定の行動後の位置を取得する。
        int x;  ///< x 要素
        int y;  ///< y 要素
    };
}
//------------------------------------------------------------------------------
// EOF
