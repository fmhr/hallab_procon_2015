//------------------------------------------------------------------------------
/// @file
/// @brief    Pos.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCPos.hpp"

#include "HPCCommon.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 原点を表すクラスのインスタンスを生成します。
    Pos::Pos()
        : x(0)
        , y(0)
    {
    }

    //------------------------------------------------------------------------------
    /// 初期値を指定してクラスのインスタンスを生成します。
    ///
    /// @param[in] aX  x成分の値
    /// @param[in] aY  y成分の値
    Pos::Pos(int aX, int aY)
        : x(aX)
        , y(aY)
    {
    }

    //------------------------------------------------------------------------------
    bool Pos::operator==(const Pos& aRhs) const
    {
        return x == aRhs.x && y == aRhs.y;
    }

    //------------------------------------------------------------------------------
    bool Pos::operator!=(const Pos& aRhs) const
    {
        return x != aRhs.x || y != aRhs.y;
    }

    //------------------------------------------------------------------------------
    const Pos Pos::move(Action aAction) const
    {
        switch (aAction) {
        case Action_MoveLeft:
            return Pos(x - 1, y);
        case Action_MoveRight:
            return Pos(x + 1, y);
        case Action_MoveDown:
            return Pos(x, y - 1);
        case Action_MoveUp:
            return Pos(x, y + 1);
        default:
            HPC_SHOULD_NOT_REACH_HERE();
            return Pos(x, y);
        }
    }
}

//------------------------------------------------------------------------------
// EOF
