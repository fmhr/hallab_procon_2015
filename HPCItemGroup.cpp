//------------------------------------------------------------------------------
/// @file
/// @brief    HPCItemGroup.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCItemGroup.hpp"

#include "HPCCommon.hpp"
#include "HPCParameter.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ItemGroup::ItemGroup()
        : mBits(0)
    {
    }

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ItemGroup::ItemGroup(int aBits)
        : mBits(aBits)
    {
    }

    //------------------------------------------------------------------------------
    /// 値を初期化します。
    void ItemGroup::reset()
    {
        mBits = 0;
    }

    //------------------------------------------------------------------------------
    /// 値を上書き設定します。
    /// 
    /// @param[in] aRhs コピー元の ItemGroup クラスへの参照。
    void ItemGroup::set(const ItemGroup& aRhs)
    {
        mBits = aRhs.mBits;
    }

    //------------------------------------------------------------------------------
    /// 荷物を追加します。
    ///
    /// @param[in] aIndex 追加する荷物のインデックス。
    void ItemGroup::addItem(int aIndex)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, Parameter::ItemCountMax);
        mBits |= 1 << aIndex;
    }

    //------------------------------------------------------------------------------
    /// 荷物を削除します。
    ///
    /// @param[in] aIndex 削除する荷物のインデックス。
    void ItemGroup::removeItem(int aIndex)
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, Parameter::ItemCountMax);
        mBits &= ~(1 << aIndex);
    }

    //------------------------------------------------------------------------------
    /// 指定インデックスの荷物が追加されているかどうかを取得します。
    ///
    /// @param[in] aIndex インデックス。
    ///
    /// @return 指定荷物が追加されているなら true、それ以外なら false。
    bool ItemGroup::hasItem(int aIndex) const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, Parameter::ItemCountMax);
        return (mBits & (1 << aIndex)) != 0;
    }

    //------------------------------------------------------------------------------
    /// 荷物が1つでも追加されているかどうかを取得します。
    ///
    /// @return 荷物が1つでも追加されているなら true、それ以外なら false。
    //------------------------------------------------------------------------------
    bool ItemGroup::hasAnyItems() const
    {
        return mBits != 0;
    }

    //------------------------------------------------------------------------------
    /// 内部状態を表すintの値を取得します。
    ///
    /// @return 内部状態を表すintの値。
    int ItemGroup::getBits() const
    {
        return mBits;
    }

    //------------------------------------------------------------------------------
    /// 内部状態を表すintの値を設定します。
    ///
    /// @param[in] aBits 内部状態を表すintの値。
    void ItemGroup::setBits(int aBits)
    {
        mBits = aBits;
    }
}

//------------------------------------------------------------------------------
// EOF
