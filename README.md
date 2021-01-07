# mc_rtc_tutorial_controller 

チュートリアル(https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/first-controller.html)を元に作った物
チュートリアル内容やいじって勉強したものごとにブランチを分けていく(予定)

# Requirement

  rosのバージョンごとに異なる（今回はmelodic環境）
  以下が必要
  https://github.com/jrl-umi3218/mc_rtc_ros
  
  詳しくは
  https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html　
  を参照の事
  

# Installation

https://jrl-umi3218.github.io/mc_rtc/tutorials/introduction/installation-guide.html　

# Usage
    /buildに移動
    $ cd build
    cmakeする
    $ cmake ../ -DCMAKE_BUILD_TYPE=RelWithDebInfo
    makeする
    $ make
    インストール
    $sudo make install 
    
    デモの実行
    $ roscore
    別ウインドウで
    $ roslaunch mc_rtc_ticker display.launch
    別ウインドウで
    $ rosrun mc_rtc_ticker mc_rtc_ticker

    うまく行かないなら、~/.config/mc_rtc/mc_rtc.yamlをみてみる
    
    {
 "MainRobot": "JVRC1",
 "Enabled": ["MyFirstController"]
}

になってるか確認

    
