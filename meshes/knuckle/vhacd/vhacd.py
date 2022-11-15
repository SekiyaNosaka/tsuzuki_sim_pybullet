# -*- coding: utf-8 -*-
# @brief pybulletのVHACDを用いたapproximate object generator

import os
import pybullet as p

_in = "../knuckle.obj"
_out = "../knuckle_approximate2.obj"
_log = "./log.txt"

p.connect(p.DIRECT)

p.vhacd(_in,
        _out,
        _log,
        #concavity=0.9, # 最大許容凹み度(default: 0.0025, range: 0.0 ~ 1.0)
        alpha=0.9, # 対称面に沿ったクリピッングの偏りを制御(default: 0.05, range: 0.0 ~ 1.0)
        beta=0.9, # 回転軸に沿ったクリピッングの偏りを制御(default: 0.05, range: 0.0 ~ 1.0)
        gamma=0.9, # マージ段階での最大許容凹凸を制御(default: 0.00125, range: 0.0 ~ 1.0)
        minVolumePerCH=0.009, # 生成された凸包の適応的なサンプリングを制御(default: 0.0001, range: 0.0 ~ 0.01)
        resolution=16000000, # ボクセル化ステージで生成されるボクセルの最大数(default: 100000, range: 10000 ~ 16000000)
        maxNumVerticesPerCH=500, # 凸包あたりの三角形の最大数を制御(default: 64, range=4 ~ 1024)
        depth=32, # クリッピングステージの最大数，各分割ステージでユーザ定義の閾値より高い凹みを持つ部分が最適なクリッピング平面に従ってクリッピングされる(default: 20, range: 1 ~ 32)
        planeDownsampling=16, # 最適クリッピング平面の探索の粒度を制御(default: 4, range: 1 ~ 16)
        convexhullDownsampling=16, # クリッピング平面選択段階での凸包生成処理の精度を制御(default: 4, range: 1 ~ 16)
        pca=True, # 凸分解を適当する前にメッシュを正規化するかどうか(default: 0, range: 0 or 1)
        #mode=1, # 0: ボクセルベースの近似凸分解, 1: 四面体ベースの近似凸分解(default: 0)
        convexhullApproximation=0, #凸包計算時の近似を有効にするか否か(default: 1, range: 0 or 1)
        #PhysicsClientID=, # 複数のサーバに接続している場合に1つを選択可能
        )
