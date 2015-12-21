//------------------------------------------------------------------------------
/// @file
/// @brief    HPCStage.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCStage.hpp"

#include "HPCAnswer.hpp"
#include "HPCCommon.hpp"
#include "HPCParameter.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Stage::Stage()
        : mField()
        , mPeriod(0)
        , mPeriodCost(0)
        , mTotalCost(0)
        , mItems()
        , mTransportStates()
        , mTruck(*this)
        , mTurnResult()
        , mTurnIndex(0)
    {
    }

    //------------------------------------------------------------------------------
    /// ステージ開始時に一度だけ呼ぶことで、ステージの初期化処理を行います。
    ///
    /// @note 関数を呼んだ後は、内部で保持している TurnResult に初期状態が記録されます。
    ///       そのため、ステージ開始後すぐに lastTurnResult() を呼ぶことで、
    ///       ステージの初期状態を取得することができます。
    void Stage::start(bool aIsInTime)
    {
        mTurnResult.state = StageState_Playing;
        mTurnIndex = 0;

        // 配達時間帯を初期化
        // 最初は-1で、時間帯ごとの初期化をするときにインクリメントされる。
        mPeriod = -1;

        // 消費した燃料を初期化
        mPeriodCost = 0;
        mTotalCost = 0;

        // 配達状況を初期化
        for (int i = 0; i < items().count(); ++i) {
            mTransportStates[i] = TransportState_NotTransported;
        }
        for (int i = items().count(); i < Parameter::ItemCountMax; ++i) {
            mTransportStates[i] = TransportState_TERM; // 存在しない荷物には TERM を設定しておく。
        }

        // トラックを初期化
        truck().reset();

        // Answerを初期化
        if (aIsInTime) {
            Answer::Init(*this);
        }

        updateTurnResult(false, Action_TERM);
    }

    //------------------------------------------------------------------------------
    /// Stage クラスに内部的に保持されているターンの実行結果を返します。
    ///
    /// @return 内部で保持されている TurnResult の値。
    const TurnResult& Stage::lastTurnResult()const
    {
        return mTurnResult;
    }

    //------------------------------------------------------------------------------
    /// Actionを実行します。
    int Stage::runAction(Action aAction)
    {
        int cost = Parameter::TruckWeight;
        for (int i = 0; i < mItems.count(); ++i) {
            if (truck().itemGroup().hasItem(i)) {
                cost += items()[i].weight();
           }
        }
        Pos nextPos = truck().pos().move(aAction);
        if (field().isWall(nextPos) == false) {
            truck().setPos(nextPos);
            // 配達できたか調べる
            for (int i = 0; i < mItems.count(); ++i) {
                if (truck().itemGroup().hasItem(i)) {
                    if (nextPos == items()[i].destination()) {
                        truck().itemGroup().removeItem(i);
                    }
                }
            }
        }

        return cost;
    }

    //------------------------------------------------------------------------------
    /// ターンを1つ進める処理を行います。
    /// トラックが空で営業所にいるなら、積み込みを行い、この配達時間帯を開始します。このターンはこれで終わりです。
    /// それ以外はトラックは移動し、配達先に着いたら自動的に荷物を降ろします。
    /// トラックが空で、移動先が営業所の場合は、この配達時間帯の終了処理を行います。ここまでは、移動と同じターンで行われます。
    void Stage::runTurn()
    {
        bool initPeriod = false;
        Action action = Action_TERM;
        if (truck().itemGroup().hasAnyItems() == false && truck().pos() == field().officePos()) {
            // トラックが空であり、かつ営業所にいるので、配達時間帯を開始する。
            mPeriod++;
            ItemGroup itemGroup;
            Answer::InitPeriod(*this, itemGroup);
            mPeriodCost = 0;

            // トラックに積み込まれた荷物が妥当か検査。
            int weight = 0;
            for (int i = 0; i < items().count(); ++i) {
                if (itemGroup.hasItem(i)) {
                    if (mTransportStates[i] != TransportState_NotTransported) {
                        // 配達済みの荷物を積み込もうとした。
                        mTurnResult.state = StageState_Failed;
                    }
                    if (items()[i].period() >= 0 && items()[i].period() != mPeriod) {
                        // この時間帯に配達すべきではない荷物を積み込んだ。
                        mTurnResult.state = StageState_Failed;
                    }
                    weight += items()[i].weight();
                    if (weight > Parameter::TruckWeightCapacity) {
                        // 最大積載重量を超えて積み込んだ。
                        mTurnResult.state = StageState_Failed;
                    }
                }
                else {
                    if (items()[i].period() == mPeriod) {
                        // この時間帯に配達すべき荷物を積み込まなかった。
                        mTurnResult.state = StageState_Failed;
                    }
                    if (mPeriod == Parameter::PeriodCount - 1 && mTransportStates[i] == TransportState_NotTransported) {
                        // 最後の時間帯なのに、残りの荷物をすべて積み込まなかった。
                        mTurnResult.state = StageState_Failed;
                    }
                }
            }
            for (int i = items().count(); i < Parameter::ItemCountMax; ++i) {
                if (itemGroup.hasItem(i)) {
                    // 存在しない荷物を積み込もうとした。
                    mTurnResult.state = StageState_Failed;
                }
            }

            if (mTurnResult.state == StageState_Failed) {
                // 積み込みに失敗した場合も、JSON出力の整合性をとるため、結果は保存する。
                updateTurnResult(true, action);
            }

            if (mTurnResult.state == StageState_Playing) {
                // トラックに積み込まれた荷物が妥当なら、この配達時間帯の配達を開始する。
                // このターンはこれで終わり。
                mTruck.itemGroup().set(itemGroup);
                for (int i = 0; i < items().count(); ++i) {
                    if (itemGroup.hasItem(i)) {
                        mTransportStates[i] = TransportState_Transporting;
                    }
                }
                initPeriod = true;
            }
            else {
                // トラックに積み込まれた荷物が妥当ではなかった場合は、直ちに終了。
                // この場合は、結果の保存も行われない。
                Answer::FinalizePeriod(*this, mTurnResult.state, 0);
                Answer::Finalize(*this, mTurnResult.state, 0);
                return;
            }
        }
        else {
            // 配達中。
            action = Answer::GetNextAction(*this);
            int cost = runAction(action);
            mPeriodCost += cost;
        }

        // 結果の保存
        updateTurnResult(initPeriod, action);

        if (truck().itemGroup().hasAnyItems() == false && truck().pos() == field().officePos()) {
            // トラックが空であり、かつ営業所にいるので、この配達時間帯を終了する。
            Answer::FinalizePeriod(*this, mTurnResult.state, mPeriodCost);
            mTotalCost += mPeriodCost;
            for (int i = 0; i < items().count(); ++i) {
                if (mTransportStates[i] == TransportState_Transporting) {
                    mTransportStates[i] = TransportState_Transported;
                }
            }
            if (mPeriod == Parameter::PeriodCount - 1) {
                mTurnResult.state = StageState_Complete;
            }
        }

        // ターン数が一定数を超えたら終了
        ++mTurnIndex;
        if (mTurnResult.state == StageState_Playing) {
            if (mTurnIndex >= Parameter::GameTurnPerStage) {
                mTurnResult.state = StageState_TurnLimit;
            }
        }

        if (mTurnResult.state != StageState_Playing) {
            // 終了
            Answer::Finalize(*this, mTurnResult.state, score());
        }
    }

    //------------------------------------------------------------------------------
    int Stage::score() const
    {
        if (mTurnResult.state == StageState_Complete) {
            return field().width() * field().height() * items().count() * 10000 / mTotalCost;
        }
        else {
            return 0;
        }
    }

    //------------------------------------------------------------------------------
    const Field& Stage::field()const
    {
        return mField;
    }

    //------------------------------------------------------------------------------
    Field& Stage::field()
    {
        return mField;
    }

    //------------------------------------------------------------------------------
    const ItemCollection& Stage::items()const
    {
        return mItems;
    }

    //------------------------------------------------------------------------------
    ItemCollection& Stage::items()
    {
        return mItems;
    }

    //------------------------------------------------------------------------------
    const Truck& Stage::truck()const
    {
        return mTruck;
    }

    //------------------------------------------------------------------------------
    Truck& Stage::truck()
    {
        return mTruck;
    }

    //------------------------------------------------------------------------------
    int Stage::period() const
    {
        return mPeriod;
    }

    //------------------------------------------------------------------------------
    TransportState Stage::getTransportState(int aIndex) const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aIndex, 0, items().count());
        return mTransportStates[aIndex];
    }

    //------------------------------------------------------------------------------
    /// TurnResultの情報を更新します。
    void Stage::updateTurnResult(bool aInitPeriod, Action aAction)
    {
        mTurnResult.initPeriod = aInitPeriod;
        mTurnResult.action = aAction;
        mTurnResult.truckPos = truck().pos();
        mTurnResult.itemGroup.set(truck().itemGroup());
        mTurnResult.periodCost = mPeriodCost;
        mTurnResult.totalCost = mTotalCost;
    }
}

//------------------------------------------------------------------------------
// EOF
