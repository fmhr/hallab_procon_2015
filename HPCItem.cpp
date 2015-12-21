//------------------------------------------------------------------------------
/// @file
/// @brief    HPCItem.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCItem.hpp"

#include "HPCCommon.hpp"
#include "HPCParameter.hpp"
#include "HPCPos.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Item::Item()
        : mDestination()
        , mPeriod(0)
        , mWeight(0)
    {
    }

    //------------------------------------------------------------------------------
    /// 値を初期化します。
    ///
    /// @param[in] aDestination 配達先の位置
    /// @param[in] aPeriod      指定時間帯。指定されてない場合は-1。
    /// @param[in] aWeight      荷物の重さ
    void Item::reset(const Pos& aDestination, int aPeriod, int aWeight)
    {
        mDestination = aDestination;
        mPeriod = aPeriod;
        mWeight = aWeight;
    }

    //------------------------------------------------------------------------------
    /// @return 配達先の座標。
    const Pos& Item::destination() const
    {
        return mDestination;
    }

    //------------------------------------------------------------------------------
    /// @return 配達時間帯。指定のない場合は-1。
    int Item::period() const
    {
        return mPeriod;
    }

    //------------------------------------------------------------------------------
    /// @return 荷物の重さ。
    int Item::weight() const
    {
        return mWeight;
    }
}

//------------------------------------------------------------------------------
// EOF
