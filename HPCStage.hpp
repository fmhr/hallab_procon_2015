//------------------------------------------------------------------------------
/// @file
/// @brief    Stage クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCField.hpp"
#include "HPCItemCollection.hpp"
#include "HPCTransportState.hpp"
#include "HPCTruck.hpp"
#include "HPCTurnResult.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// ゲームの1ステージを表します。
    class Stage 
    {
    public:
        Stage();

        ///@name ステージの実行
        //@{
        void start(bool aIsInTime);                     ///< ステージを開始します。
        void runTurn();                                 ///< ターンを1つ進めます。
        const TurnResult& lastTurnResult()const;        ///< 最後のターン実行後の結果を返します。
        //@}

        /// @name 各要素へのアクセス
        //@{
        const Field& field()const;          ///< フィールド情報を返します。
        Field& field();                     ///< フィールド情報を返します。
        const ItemCollection& items()const; ///< 荷物情報を返します。
        ItemCollection& items();            ///< 荷物情報を返します。
        const Truck& truck()const;          ///< トラック情報を返します。
        Truck& truck();                     ///< トラック情報を返します。
        int period() const;                 ///< 現在の配達時間帯を返します。
        TransportState getTransportState(int aIndex) const; ///< 指定荷物の配達状況を取得します。
        //@}
        int score() const;                  ///< スコアを返します。

    private:
        int runAction(Action aAction); ///< Actionを実行します。

        Field mField;                   ///< フィールド情報
        int mPeriod;                    ///< 配達時間帯。0 ～ PeriodCount - 1
        int mPeriodCost;                ///< この時間帯に消費した燃料。
        int mTotalCost;                 ///< この時間帯までに消費した燃料。
        ItemCollection mItems;          ///< 荷物情報
        TransportState mTransportStates[Parameter::ItemCountMax]; ///< 配達状況
        Truck mTruck;                   ///< トラック情報
        TurnResult mTurnResult;         ///< ターンの実行結果
        int mTurnIndex;                 ///< 現在のターン番号

        void updateTurnResult(bool aInitPeriod, Action aAction); ///< TurnResultを更新します。
    };
}
//------------------------------------------------------------------------------
// EOF
