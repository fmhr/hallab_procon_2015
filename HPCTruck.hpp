//------------------------------------------------------------------------------
/// @file
/// @brief    Truck クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCItemGroup.hpp"
#include "HPCParameter.hpp"
#include "HPCPos.hpp"

namespace hpc {

    class Stage;

    //------------------------------------------------------------------------------
    /// トラックを表します。
    ///
    class Truck
    {
    public:
        Truck(const Stage& aStage);

        void reset();                   ///< 初期化

        const Pos& pos() const;         ///< 現在位置を取得する
        void setPos(const Pos& aPos);   ///< 現在位置を設定する
        const ItemGroup& itemGroup() const; ///< 荷物グループを取得する。
        ItemGroup& itemGroup();             ///< 荷物グループを取得する。

    private:
        const Stage& mStage;      ///< ステージ
        Pos mPos;           ///< トラックの位置
        ItemGroup mItemGroup;
    };
}
//------------------------------------------------------------------------------
// EOF
