//------------------------------------------------------------------------------
/// @file
/// @brief    HPCLevelDesigner.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCLevelDesigner.hpp"

#include <cstdlib>
#include "HPCCommon.hpp"
#include "HPCMath.hpp"
#include "HPCRandom.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 渡された Stage に対しマップを生成します。
    ///
    /// @param[in]      aNumber ステージ番号
    /// @param[in,out]  aStage  ステージ情報。関数を呼ぶと書き換えられます。
    /// @param[in,out]  aRandom 乱数
    void LevelDesigner::Setup(int aNumber, Stage& aStage, Random& aRandom)
    {
        int width = Parameter::FieldWidthMin;
        width += aRandom.randTerm((Parameter::FieldWidthMax - Parameter::FieldWidthMin) / 4 + 1) * 4;
        int height = Parameter::FieldHeightMin;
        height += aRandom.randTerm((Parameter::FieldHeightMax - Parameter::FieldHeightMin) / 4 + 1) * 4;

        // ステージ番号から、壁密度、時間帯指定されている荷物の割合、荷物数を決める。
        int wallDensityIndex = aNumber % Parameter::WallDensityMax;
        int periodSpecifiedIndex = (aNumber / Parameter::WallDensityMax) % Parameter::PeriodSpecifiedMax;
        int itemCountIndex = (aNumber / (Parameter::WallDensityMax * Parameter::PeriodSpecifiedMax)) % Parameter::ItemCountMax;

        // wallDensity は 0 にはならないようにする。あまりにも壁がない迷路になるため。
        int wallDensity = (wallDensityIndex + 1) * (100 / Parameter::WallDensityMax);

        // 時間帯指定されている荷物の個数。割合がそのまま適用される。端数は切り捨て。
        int itemCount = itemCountIndex + 1;
        int periodSpecifiedCount = itemCount * periodSpecifiedIndex / (Parameter::PeriodSpecifiedMax - 1);

        // フィールド生成
        aStage.field().setup(width, height, wallDensity, aRandom);

        // 荷物の生成
        aStage.items().reset();
        int weightHistogram[Parameter::ItemWeightMax + 1] = { 0 };
        int itemWeights[Parameter::ItemCountMax];
        int periodSpecs[Parameter::ItemCountMax];
        for (int i = 0; i < itemCount; ++i) {
            int w;
            do {
                w = aRandom.randMinMax(Parameter::ItemWeightMin, Parameter::ItemWeightMax);
            } while (weightHistogram[w] >= Parameter::WeightHistogramMax);
            itemWeights[i] = w;
            weightHistogram[w]++;
            periodSpecs[i] = -1;
        }

        // 時間帯指定を決める。
        int periodItemWeightSum[Parameter::PeriodCount] = { 0 };
        for (int i = 0; i < periodSpecifiedCount; ++i) {
            for (;;) {
                int ix = aRandom.randTerm(itemCount);
                if (periodSpecs[ix] == -1) {
                    int p = aRandom.randTerm(Parameter::PeriodCount);
                    if (periodItemWeightSum[p] + itemWeights[ix] <= Parameter::TruckWeightCapacity) {
                        periodSpecs[ix] = p;
                        periodItemWeightSum[p] += itemWeights[ix];
                        break;
                    }
                }
            }
        }

        // 荷物を生成
        for (int i = 0; i < itemCount; i++) {
            Pos pos;
            for (;;) {
                int x = aRandom.randTerm(width);
                int y = aRandom.randTerm(height);
                pos = Pos(x, y);
                if (aStage.field().isWall(pos)) {
                    continue;
                }
                if (aStage.field().officePos() == pos) {
                    continue;
                }
                bool overlaps = false;
                for (int j = 0; j < i; ++j) {
                    if (aStage.items()[j].destination() == pos) {
                        overlaps = true;
                        break;
                    }
                }
                if (overlaps) {
                    continue;
                }
                break;
            }
            aStage.items().addItem(pos, periodSpecs[i], itemWeights[i]);
        }
    }
}

//------------------------------------------------------------------------------
// EOF
