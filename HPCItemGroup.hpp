//------------------------------------------------------------------------------
/// @file
/// @brief    ItemGroup クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

namespace hpc {

    //------------------------------------------------------------------------------
    /// 荷物のインデックスの組み合わせを表します。
    class ItemGroup
    {
    public:
        ItemGroup();
        explicit ItemGroup(int aBits);

        void reset(); ///< 情報を既定の値で初期化します。
        void set(const ItemGroup& aRhs); ///< 値を上書き設定します。

        void addItem(int aIndex);        ///< 指定インデックスの荷物を積み込む
        void removeItem(int aIndex);     ///< 指定インデックスの荷物を降ろす
        bool hasItem(int aIndex) const; ///< 指定インデックスの荷物が積み込まれているかどうかを取得する
        bool hasAnyItems() const;       ///< 荷物が1つでも積み込まれているかどうかを取得する

        int getBits() const;            ///< ビットマスクを取得する
        void setBits(int aBits);        ///< ビットマスクを設定する

    private:
        int mBits;
    };
}
//------------------------------------------------------------------------------
// EOF
