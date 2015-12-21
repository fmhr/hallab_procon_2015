//------------------------------------------------------------------------------
/// @file
/// @brief    HPCItemCollection.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCItemCollection.hpp"

#include "HPCCommon.hpp"
#include "HPCStage.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ItemCollection::ItemCollection()
        : mItems()
        , mCount(0)
    {
    }

    //------------------------------------------------------------------------------
    /// 荷物のデータを設定します。
    /// 
    /// @param[in] aRhs コピー元の ItemCollection クラスへの参照。
    void ItemCollection::set(const ItemCollection& aRhs)
    {
        reset();

        for (int index = 0; index < aRhs.count(); ++index) {
            mItems[index] = aRhs[index];
        }
        mCount = aRhs.mCount;
    }

    //------------------------------------------------------------------------------
    /// 荷物のデータを初期化し、初期状態に戻します。
    /// 有効な荷物数は 0 となります。
    void ItemCollection::reset()
    {
        mCount = 0;
    }

    //------------------------------------------------------------------------------
    /// 荷物を追加します。
    ///
    void ItemCollection::addItem(const Pos& aDestination, int aPeriod, int aWeight)
    {
        mItems[mCount++].reset(aDestination, aPeriod, aWeight);
    }

    //------------------------------------------------------------------------------
    /// @return 現在有効な荷物数
    int ItemCollection::count()const
    {
        return mCount;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aIndex 有効な荷物のインデックス。
    ///
    /// @return aIndex 番目の有効な荷物への参照を返します。
    const Item& ItemCollection::operator[](int aIndex)const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, mCount);
        return mItems[aIndex];
    }

    //------------------------------------------------------------------------------
    /// @param[in] aIndex 有効な荷物のインデックス。
    ///
    /// @return aIndex 番目の有効な荷物への参照を返します。
    Item& ItemCollection::operator[](int aIndex)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, mCount);
        return mItems[aIndex];
    }
}

//------------------------------------------------------------------------------
// EOF
