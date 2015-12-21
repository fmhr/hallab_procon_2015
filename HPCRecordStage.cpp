//------------------------------------------------------------------------------
/// @file
/// @brief    HPCRecordStage.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCRecordStage.hpp"

#include "HPCCommon.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    RecordStage::RecordStage()
        : mCurrentTurn(0)
        , mScore(0)
#ifdef DEBUG
        , mTurns()
        , mField()
        , mItems()
#endif
    {
    }

    //------------------------------------------------------------------------------
    /// ステージの記録を開始することを通知します。
    ///
    /// @param[in] aStage 現在実行しているステージを表す Stage クラスへの参照。
    void RecordStage::writeStart(const Stage& aStage)
    {
#ifdef DEBUG
        mField.set(aStage.field());
        mItems.set(aStage.items());
#endif
    }

    //------------------------------------------------------------------------------
    /// 毎ターンの記録を行います。
    ///
    /// @param[in] aResult 現在のターンを表す実行結果。
    void RecordStage::writeTurn(const TurnResult& aResult)
    {
#ifdef DEBUG
        HPC_RANGE_ASSERT_MIN_UB_I(mCurrentTurn, 0, HPC_ARRAY_NUM(mTurns));
        mTurns[mCurrentTurn].set(aResult);
#endif
        ++mCurrentTurn;
    }

    //------------------------------------------------------------------------------
    /// 終了時の記録を行います。
    ///
    /// @param[in] aStage 現在実行しているステージを表す Stage クラスへの参照。
    void RecordStage::writeEnd(const Stage& aStage)
    {
        // スコアを計算
        mScore = aStage.score();
    }

    //------------------------------------------------------------------------------
    /// @return ステージ毎の得点。
    int RecordStage::score()const
    {
        return mScore;
    }

    //------------------------------------------------------------------------------
    void RecordStage::dumpItemGroup(ItemGroup aItemGroup) const
    {
        bool first = true;
        for (int j = 0; j < Parameter::ItemCountMax; ++j) {
            if (aItemGroup.hasItem(j)) {
                if (first == false) {
                    HPC_PRINT(",");
                }
                first = false;
                HPC_PRINT("%d", j);
            }
        }
    }

    //------------------------------------------------------------------------------
    char RecordStage::encodePosInt(int a) const
    {
        if (a < 10) {
            return a + '0';
        }
        else if (a < 36) {
            return a - 10 + 'A';
        }
        else {
            HPC_SHOULD_NOT_REACH_HERE();
            return '0';
        }
    }

    //------------------------------------------------------------------------------
    /// 記録された結果を画面に出力します。
    void RecordStage::dump()const
    {
#ifdef DEBUG
        HPC_PRINT_LOG(
            "Field", "(%d,%d)\n", mField.width(), mField.height()
            );
        for (int i = mField.height() - 1; i >= 0; --i) {
            for (int j = 0; j < mField.width(); ++j) {
                Pos pos(j, i);
                bool flag = false;
                for (int k = 0; k < mItems.count(); ++k) {
                    if (mItems[k].destination() == pos) {
                        HPC_PRINT("%02d", k);
                        flag = true;
                        break;
                    }
                }
                if (flag == false) {
                    if (mField.isWall(pos)) {
                        HPC_PRINT("[]");
                    }
                    else {
                        HPC_PRINT("  ");
                    }
                }
            }
            HPC_PRINT("\n");
        }
        for (int i = 0; i < mItems.count(); ++i) {
            HPC_PRINT_LOG("Item", "%2d: (%2d,%2d) period=%d, weight=%d\n", i, mItems[i].destination().x, mItems[i].destination().y, mItems[i].period(), mItems[i].weight());
        }
        ItemGroup itemGroup;
        int initPeriodCount = 0;
        for (int i = 0; i < mCurrentTurn; ++i) {
            bool showTurn = false;
            if (mTurns[i].initPeriod) {
                ++initPeriodCount;
                showTurn = true;
            }
            if (i > 0 && mTurns[i].itemGroup.getBits() != mTurns[i - 1].itemGroup.getBits()) {
                showTurn = true;
            }
            if (i > 0 && i + 1 < mCurrentTurn && mTurns[i + 1].initPeriod) {
                showTurn = true;
            }
            if (i == mCurrentTurn - 1) {
                showTurn = true;
            }
            if (showTurn) {
                itemGroup.set(mTurns[i].itemGroup);
                // 表示上のターン数は、積み込みターンは含めないようにします。
                HPC_PRINT_LOG("Turn", "#%04d: period=%d,periodCost=%d,totalCost=%d,items=", i - 1, initPeriodCount - 1, mTurns[i].periodCost, mTurns[i].totalCost + mTurns[i].periodCost);
                dumpItemGroup(itemGroup);
                HPC_PRINT("\n");
            }
        }
#endif
        HPC_PRINT_LOG("Score", "%d\n", static_cast<int>(score()));
    }

    //------------------------------------------------------------------------------
    /// 記録された結果をJSON形式で出力します。
    ///
    /// @param[in] isCompressed 圧縮された状態で出力するかどうか。
    ///                         @c true にすると、改行やインデントを除いた形で出力されます。
    void RecordStage::dumpJson(bool isCompressed)const
    {
#ifdef DEBUG
        HPC_PRINT_JSON_DEBUG(!isCompressed, "    "); // インデント (4)
        HPC_PRINT("[");
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

        // 営業所の位置
        HPC_PRINT_JSON_DEBUG(!isCompressed, "      "); // インデント (6)
        HPC_PRINT("%d,%d,", mField.officePos().x, mField.officePos().y);
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

        // フィールド
        HPC_PRINT_JSON_DEBUG(!isCompressed, "      "); // インデント (6)
        HPC_PRINT("[");
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
        for (int i = 0; i < mField.height(); ++i) {
            HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
            HPC_PRINT("\"");
            for (int j = 0; j < mField.width(); ++j) {
                HPC_PRINT("%d", mField.isWall(j, i) ? 1 : 0);
            }
            HPC_PRINT("\"");
            if (i + 1 < mField.height()) {
                HPC_PRINT(",");
            }
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
        }
        HPC_PRINT_JSON_DEBUG(!isCompressed, "      "); // インデント (6)
        HPC_PRINT("],");
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

        // 荷物
        HPC_PRINT_JSON_DEBUG(!isCompressed, "      "); // インデント (6)
        HPC_PRINT("[");
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
        for (int i = 0; i < mItems.count(); ++i) {
            HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
            HPC_PRINT("[%d,%d,%d,%d]", mItems[i].destination().x, mItems[i].destination().y, mItems[i].period() + 1, mItems[i].weight());
            if (i + 1 < mItems.count()) {
                HPC_PRINT(",");
            }
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
        }
        HPC_PRINT_JSON_DEBUG(!isCompressed, "      "); // インデント (6)
        HPC_PRINT("],");
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

        // スコア
        HPC_PRINT_JSON_DEBUG(!isCompressed, "      "); // インデント (6)
        HPC_PRINT("%d,", mScore);
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");

        // 配達時間
        int period = -1;
        bool periodFirst = true;
        for (int i = 1; i < mCurrentTurn; ++i) { // 初期状態は不要なので1から始める
            if (mTurns[i].initPeriod) {
                period++;
                if (period > 0) {
                    // 前の配達時間帯終了
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
                    HPC_PRINT("]");
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "      "); // インデント (6)
                    HPC_PRINT("],");
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                }
                // 配達時間帯開始
                HPC_PRINT_JSON_DEBUG(!isCompressed, "      "); // インデント (6)
                HPC_PRINT("[");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
                HPC_PRINT("[");
                bool itemFirst = true;
                for (int j = 0; j < Parameter::ItemCountMax; ++j) {
                    if (mTurns[i].itemGroup.hasItem(j)) {
                        if (itemFirst == false) {
                            HPC_PRINT(",");
                        }
                        itemFirst = false;
                        int deliveryTurn = mCurrentTurn;
                        for (int k = i + 1; k < mCurrentTurn; ++k) {
                            if (mTurns[k].itemGroup.hasItem(j) == false) {
                                deliveryTurn = k - 1; // 初期状態の分を引く
                                break;
                            }
                        }
                        HPC_PRINT("[%d,%d]", j, deliveryTurn);
                    }
                }
                HPC_PRINT("],");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
                HPC_PRINT("[");
                HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                periodFirst = true;
            }
            {
                if (periodFirst == false) {
                    HPC_PRINT(",");
                    HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
                }
                periodFirst = false;
                HPC_PRINT_JSON_DEBUG(!isCompressed, "          "); // インデント (10)
                HPC_PRINT("\"%c%c%X\"", encodePosInt(mTurns[i].truckPos.x), encodePosInt(mTurns[i].truckPos.y), (mTurns[i].totalCost + mTurns[i].periodCost));
            }
        }
        // 最後の配達時間帯終了
        if (period >= 0) {
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "        "); // インデント (8)
            HPC_PRINT("]");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
            HPC_PRINT_JSON_DEBUG(!isCompressed, "      "); // インデント (6)
            HPC_PRINT("]");
        }
        HPC_PRINT_JSON_DEBUG(!isCompressed, "\n");
        HPC_PRINT_JSON_DEBUG(!isCompressed, "    "); // インデント (4)
        HPC_PRINT("]");
#else
        // デバッグ無効の場合、json 出力はサポートされません。
        HPC_PRINT("[]");
#endif
    }
}

//------------------------------------------------------------------------------
// EOF
