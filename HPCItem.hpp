//------------------------------------------------------------------------------
/// @file
/// @brief    Item クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCPos.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 荷物を表します。
    class Item
    {
    public:
        Item();

        void reset(const Pos& aDestination, int aPeriod, int aWeight); ///< 情報を既定の値で初期化します。

        const Pos& destination()const; ///< 配達先の位置を返します。
        int period()const;             ///< 指定時間帯を返します。時間帯が指定されてない場合は-1を返します。
        int weight()const;             ///< 荷物の重さを返します。

    private:
        Pos mDestination;
        int mPeriod;
        int mWeight;
    };
}
//------------------------------------------------------------------------------
// EOF
