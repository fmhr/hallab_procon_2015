//------------------------------------------------------------------------------
/// @file
/// @brief    ItemCollection クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCItem.hpp"
#include "HPCParameter.hpp"

namespace hpc {

    class Stage;
    
    //------------------------------------------------------------------------------
    /// 荷物のセットを表します。
    ///
    /// 1ステージのすべての荷物は、この ItemCollection クラスに格納されます。
    class ItemCollection
    {
    public:
        ItemCollection();

        void set(const ItemCollection& aRhs);               ///< 荷物データを設定します。
        void reset();                                       ///< 荷物データを初期化します。
        void addItem(const Pos& aDestination, int aPeriod, int aWeight);///< 荷物を追加します。

        int count()const;                                   ///< 有効な荷物数を返します。

        /// @name 有効な荷物へのアクセス
        //@{
        const Item& operator[](int aIndex)const;
        Item& operator[](int aIndex);
        //@}

    private:
        Item mItems[Parameter::ItemCountMax];   ///< 荷物用配列
        int mCount;                                 ///< 荷物数
    };
}
//------------------------------------------------------------------------------
// EOF
