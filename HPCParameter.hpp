//------------------------------------------------------------------------------
/// @file
/// @brief    Parameter クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

namespace hpc {

    //------------------------------------------------------------------------------
    /// @brief ゲームに関するパラメータを表します。
    class Parameter
    {
    public:
        ///@name ゲームルール
        //@{
        static const int GameTurnPerStage = 1000;       ///< 1ステージあたりのゲーム時間
        static const int WallDensityMax = 5;            ///< 壁密度の段階数
        static const int PeriodSpecifiedMax = 6;        ///< 時間帯指定されている荷物の割合の段階数
        static const int ItemCountMax = 16;             ///< 1ステージの荷物の最大数
        static const int RepeatCount = 1;               ///< 繰り返し回数。コンテスト開催中は1だが、締め切り後の再評価で4に変更される。
        static const int GameStageCount = WallDensityMax * PeriodSpecifiedMax * ItemCountMax * RepeatCount; ///< ステージ数
        static const int GameTimeLimitSec = 0;          ///< 制限時間(秒)。0なら制限時間なし。
        //@}

        ///@name ステージ
        //@{
        static const int PeriodCount = 4;               ///< 配達時間帯数
        static const int FieldWidthMin = 19;            ///< 最小フィールド幅(4で割ると3余る)
        static const int FieldHeightMin = 19;           ///< 最小フィールド高さ(4で割ると3余る)
        static const int FieldWidthMax = 31;            ///< 最大フィールド幅(4で割ると3余る)
        static const int FieldHeightMax = 31;           ///< 最大フィールド高さ(4で割ると3余る)
        static const int PeriodAllMask = (1 << PeriodCount) - 1;
        //@}

        ///@name トラック
        //@{
        static const int TruckWeightCapacity = 15;      ///< トラックの最大積載重量
        static const int TruckWeight = 3;               ///< トラックの重さ
        //@}

        ///@name 荷物
        //@{
        static const int ItemWeightMin = 1;             ///< 荷物の重さの最小値
        static const int ItemWeightMax = 3;             ///< 荷物の重さの最大値
        static const int WeightHistogramMax = (ItemCountMax + (ItemWeightMax - ItemWeightMin + 1) - 1) / (ItemWeightMax - ItemWeightMin + 1); ///< 同じ重さの荷物の個数の最大値
        //@}

    private:
        Parameter();
    };
}
//------------------------------------------------------------------------------
// EOF
