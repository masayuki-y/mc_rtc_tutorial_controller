実装内容は箱のリーチングモーション

箱の配置
->ロボットの足元をワールド座標原点に設定し、ワールド座標で(0.6, 0.0, 1.2)に配置

EFの配置
->左手は{0.5, 0.15, 1.2}、右手は{0.5, -0.15, 1.2}に移動

CoMの目標値
->シミュレーション開始の値を目標値として設定
　開始時の値はlogが取れる？


test3では大幅に偏向
→手の位置 (0.5, -0.135, 1.2) (0.5, 0.135, 1.2)
→箱の配置　(0.65, 0.0, 1.2)

開始時のCoMを目標値に設定、その値を保持するように調整する関数を使用

//comZero is obtained by doing:
comZero = comTask->com();


//comが下がったら、重心位置を上げるように調整
  if(comDown)
  {
    comTask->com(comZero - Eigen::Vector3d{0, 0, -0.2});
  }
  else
  {
    comTask->com(comZero);
  }
  comDown = !comDown;
}


test4 物体の位置を変更　(下方)
→箱の座標　(0.65, 0.0, 0.6)
→手の位置  (0.55, -0.135, 0.55) (0.55, 0.135, 0.55)

com test3のままだとしゃがめなかったので、変更
→comがさげられるように調節
    comTask->com(comZero - Eigen::Vector3d{0, 0, 0.1});
  }
  else
  {
    comTask->com(comZero);
  }
  comDown = !comDown;
}
