//------------------------------------------------------------------------------
/// @file
/// @brief    HPCTruck.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCTruck.hpp"

#include "HPCCommon.hpp"
#include "HPCStage.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Truck::Truck(const Stage& aStage)
        : mStage(aStage)
        , mPos()
        , mItemGroup()
    {
    }

    //------------------------------------------------------------------------------
    /// リセットします。
    void Truck::reset()
    {
        mPos = mStage.field().officePos();
        mItemGroup.reset();
    }

    //------------------------------------------------------------------------------
    /// @return トラックの位置
    const Pos& Truck::pos() const
    {
        return mPos;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aPos トラックの位置を設定します。
    void Truck::setPos(const Pos& aPos)
    {
        mPos = aPos;
    }

    //------------------------------------------------------------------------------
    /// @return ItemGroupを取得します。
    const ItemGroup& Truck::itemGroup() const
    {
        return mItemGroup;
    }

    //------------------------------------------------------------------------------
    /// @return ItemGroupを取得します。
    ItemGroup& Truck::itemGroup()
    {
        return mItemGroup;
    }
}

//------------------------------------------------------------------------------
// EOF
