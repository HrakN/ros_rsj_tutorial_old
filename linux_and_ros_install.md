---
title: Ubuntu LinuxとROSのインストール
date: 2021-01-12
---

- Table of contents
{:toc}

本セミナーで使用する開発環境として Ubuntu Linux とその上で動作する ROS を利用します。<br>
本ページでは Ubuntu Linux と ROS のインストール方法を紹介します。

## 用意するもの

- ノート型パソコンあるいはデスクトップPC

  *<span style="color: red">本手順によりパソコンの既存のOS（Windows等）及び保存されているデータやソフトウェアは完全に削除されます。予めバックアップを行ってください。</span>*
- 容量 4GB 以上の空 USB メモリ
- インターネット接続
- モニター１台
- キーボード
- HDMIケーブル
- マウス
- 容量 4GB 以上の空 SDカード

## リモートPCセットアップ

### Ubuntu Linux のダウンロード

下記URLから Ubuntu 18.04 64bit Desktopのインストールイメージ、`ubuntu-18.04.x-desktop-amd64.iso`(xはバージョン番号)をダウンロードします。

   [Ubuntu 18.04 Bionic](http://jp.releases.ubuntu.com/bionic/)

* 本セミナーでは Ubuntu 18.04 64bit Desktop 版の使用を想定し、説明します
* ネットワーク環境によっては、インストールイメージのリンクをクリック後、実際にダウンロード開始するまで、10分ほど必要な場合があるようです
  <!-- https://ubuntu-news.org/2021/09/17/ubuntu-18-04-6-lts-released/ -->
   
   ![Ubuntu ダウンロード](images/seminar_no139/download_ubuntu18.04_0.png)


### Live USB の作成

1. 下記 URL から、Live USB 作成ソフトをダウンロードします。
   - Windows、Mac OS Xの場合：<https://unetbootin.github.io/>

     ページ中の、Live USB 作成に使用しているPCのOSを選択してください。

     ![UNetbootinダウンロード](images/unetbootin_download.png)

   - Linux (Debian/Ubuntu) の場合: 下記コマンドを実行

     ```shell
     $ sudo add-apt-repository ppa:gezakovacs/ppa
       ("[ENTER]を押すと続行します。"の表示後、Enterを入力)
     $ sudo apt-get update
     $ sudo apt-get install unetbootin
     ```

     <b>以降の説明は、Windowsを元に行います。</b>

1. 誤って必要なデータを削除してしまうのを防ぐため、使用しない USB メモリや、メモリーカードを取り外し、使用する USB メモリのみを接続します。<br>
使用する USB メモリは、ファイルが入っていない空の状態にして下さい。

   (Live USB を作成する PC と Ubuntu をインストール する PC はそれぞれ別でも構いません)

1. ダウンロードした unetbootin-windows-???.exe（Windows の場合）を実行します。<br>
   下記、「Windows によって PC が保護されました」画面が現れた場合は、「実行」ボタンをクリックしてください。

   ![Windows UAC](images/windows_idiot_screen.png)

   また、下記のユーザアカウント制御画面が現れた場合、「はい」をクリックしてください。

   ![Windows UAC](images/windows_uac.png)

1. UNetbootin の画面で、「ディスクイメージ」を選択し、「…」ボタンをクリックして先ほどダウンロードした`ubuntu-18.04.x-desktop-amd64.iso` ファイルを選択します。(x はダウンロードしたファイルに合わせて変更ください)<br>

    また、「スペースは、リブートしてもファイルを維持するために使用」欄に「4096」と入力し、「ドライブ」欄で、使用する USB メモリのドライブ名を選択します。<br>
  内容を確認後、「OK」をクリックしてください。

   ![UNetbootin settings](images/seminar_no139/unetbootin_settings.png)

   書き込み完了までしばらく待機します。<br>
   USB2.0 の場合10分以上、書き込み速度の遅いメモリだと30分程度かかる場合があります。<br>

   また、以下のように、文字が描画されていないウィンドウが表示されることがあるようですが、ウィンドウ右上の「x」ボタンにより、ウィンドウを閉じても問題ないようです。

   ![UNetbootin empty window](images/seminar_no139/unetbootin_empty_window.png)

   下記の「永続性を設定する」画面で「応答なし」と表示される場合がありますが、正常に動作していますので、そのまま待機してください。

   ![UNetbootin may freeze](images/seminar_no139/unetbootin_may_freeze.png)

   下記画面が表示されれば、「Live USB」の作成は完了です。終了をクリックして下さい。

   ![UNetbootin complete](images/seminar_no139/unetbootin_complete_bootmemory.png)

1. Live USBから起動するためのBIOSの設定を行います。

   セミナーで使用する PC の電源を切り、下記の手順で作成した Live USB を接続した状態で起動します。<br>
   起動時に、BIOS 設定画面に入ります。<span style="color: red">*PC のメーカー毎に BIOS への入り方が異なります*</span>ので、マニュアル等で確認してください。<br>
   下記の図は Acer での BIOS 設定画面の例です。

   ![BIOS 1](images/acer_boot.png)

   BIOS 設定画面に入ったら、起動順（Boot order, Boot priority）の設定で、USB メモリが最優先になるように設定します。 （表示は使用している PC および USB メモリのメーカーによって異なります）

   ![BIOS 2](images/acer_boot_order1.png)

   ![BIOS 3](images/acer_boot_order2.png)

   設定を保存して再起動します。

   ![BIOS 4](images/acer_boot_save.png)

### Ubuntu Linux のインストール

1. Live USB をパソコンに接続し、パソコンの電源を入れます。

1. "GNU GRUB"のメニューの表示時、"Install Ubuntu"を選択します。

1. 以下の画面では言語を選択し、"続ける"を押します。その後、キーボードやネットワークに関しても環境に合わせて設定ください。

   ![Ubuntu install 1](images/seminar_no139/ubuntuinst_0_language.png)

1. 以下の設定では、「通常のインストール」を選択してください。

   ![Ubuntu install 2](images/seminar_no139/ubuntuinst_3_software.png)

1. 以下の設定では、「ディスクを削除して Ubuntu をインストールする」を選択してください。

   ![Ubuntu install 3](images/seminar_no139/ubuntuinst_4_filesystem.png)

1. インストール完了後、<span style="color: red">*LiveUSBを未接続の状態で*</span>パソコンを再起動すると以下の画面が現れます。<br>
これで Ubuntu Linux のインストールが完了です。

   ![Ubuntu install 4](images/seminar_no139/ubuntuinst_7_desktop.png)


### ROS Melodic ベースパッケージのインストール

![](images/turtlebot3/remote_pc_and_turtlebot.png)

>**注意**
>
>本章の内容は、TurtleBot3を制御する`リモートPC`(Ubuntu18.04がインストールされたデスクトップまたは、ノートパソコン)に対応しています。 **この手順は、TurtleBot3で実施しないでください。**


下記のスクリプトを使用すると、ROS Melodic(ROS)のインストール手順を簡略化できます。
ターミナルウィンドウでこのスクリプトを実行します。ターミナルアプリケーションは、画面の左上隅にあるUbuntu検索アイコンから起動できます。もしくは、ターミナルのショートカットキー`Ctrl+Alt+t`を使用して起動できます。 ROSをインストールした後、リモートPCを再起動してください。
<br><br>

*【リモートPCで実施】*
```shell
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
$ chmod 755 ./install_ros_melodic.sh 
$ bash ./install_ros_melodic.sh
```

**注釈**: インストールされるパッケージを確認するには、次のリンクを確認してください。

[install_ros_melodic.sh](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh)


上記のインストールが失敗した場合、以下を参考にしてください。
<!-- the words above are described in https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup -->
[公式ROS1Melodicインストールガイド](http://wiki.ros.org/melodic/Installation/Ubuntu)

<!-- 
**注釈**:  
 - ROBOTISのROSパッケージはMelodic Moreniaをサポートしていますが、TurtleBot3にはROS Kinetic Kameを使用することを推奨します。
 - ROSをMelodic Moreniaにアップグレードする場合は、サードパーティのROSパッケージが完全にサポートされていることを確認してください。
<div class ="notice--info">{{info_01 | markdownify}}</div>
-->

### ROS 依存パッケージのインストール

リモートPCにROS依存パッケージをインストールする手順です。<br><br>

*【リモートPCで実施】*
```shell
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```

リモートPCにTurtleBot3を制御するための依存パッケージをインストールする手順です。
<br><br>

*【リモートPCで実施】*
```shell
$ sudo apt-get install ros-melodic-dynamixel-sdk
$ sudo apt-get install ros-melodic-turtlebot3-msgs
$ sudo apt-get install ros-melodic-turtlebot3
```

### TurtleBot3モデル名を設定

環境変数`TURTLEBOT3_MODEL`モデルにデフォルト名を設定します。以下のコマンドを端末に入力し、環境変数の設定と反映を行います。

*【リモートPCで実施】*
```shell
$ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
$ source ~/.bashrc
```

`catkin_make`コマンドへのパスが通っていることを確認します。<br>
以下のコマンドを実行後、"/opt/ros/melodic/bin/catkin_make"といったパスが表示されれば、正しく設定できています。

*【リモートPCで実施】*
```shell
$ which catkin_make
```

何も表示されない場合、環境変数設定に不備が考えられるため、以下のコマンドを再実行してください。


*【リモートPCで実施】*
```shell
$ source ~/.bashrc
```

<a id="remotepc_wifi_settings"></a>
### ネットワーク構成

![](images/turtlebot3/network_configuration.png)

TurtleBot PCとリモートPCの間で通信をするためにIPアドレスが必要です。 **リモートPCとTurtleBot PCは、同じwifiルーターに接続する必要があります。**

`リモートPC`のターミナルウィンドウで次のコマンドを入力し、リモートPCのIPアドレスを確認します。
<br>

*【リモートPCで実施】*
```shell
$ ip address show
```

赤枠部分が、`リモートPC`のIPアドレスです。
(環境によってIPアドレスが異なる場合があります。また、"/24"の部分は含みません。)

![](images/seminar_no139/rosset_0_ip.png)

以下のコマンドを入力し、~/.bashrcを開きます。

*【リモートPCで実施】*
```shell
$ nano ~/.bashrc
```

`Alt + /` を入力するとファイルの最終行へ移動します。

`ROS_MASTER_URI`と`ROS_HOSTNAME`の`localhost`のIPアドレスを、上記のターミナルウィンドウから取得したIPアドレスに変更します。
(下図、赤枠部分)

![](images/seminar_no139/rosset_1_env.png)

修正後、ファイルを保存(`Ctrl + o` → `enter`)し、編集を終了（`Ctrl + x`）します。

次に、以下のコマンドでbashrcを実行します。

*【リモートPCで実施】*
```shell
$ source ~/.bashrc
```

以上でリモートPCの開発環境構築は完了です。


## 手順（TurtleBot制御コンピューター：Raspberry Pi 3+ セットアップ）


>**警告**
>- この章の内容は、**TurtleBot3 Burger**の制御コンピューターとなる `Raspberry Pi 3+(Raspberry Pi)`に対応しています。 **本項の設定内容をリモートPC（デスクトップPCまたは、ノートパソコン）で実施しないでください。**
>- セットアップ作業には、電源と時間が必要なためバッテリーは適していません。この作業では、SMPS(ACアダプタ)の使用を推奨します。


139回ロボット工学セミナー経由で実習キットを購入された方は、付属のSDカードに適切なイメージが記録されているので、下の[WiFiネットワーク設定を構成する](#tb3_wifi_settings)から実施ください。

もし、お持ちのTurtleBot3 BurgerにROS Melodicがインストールされていない場合、以下のHPより、Raspberry PiにROS Melodicのインストールを実施する必要があります。(画面上部のROSバージョン選択箇所において、'Melodic'を選択ください)

 3.2.1 "microSDカードとリーダーを準備する" ～ 3.2.4.2 "ディスクユーティリティ" まで実施ください。

https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup


<!-- ページ内リンクのおまじない -->
<a id="tb3_wifi_settings"></a> 
### WiFiネットワーク設定を構成する

1. TurtleBotの制御コンピュータであるRaspberry Piを起動

   以下の手順で、TurtleBotの制御コンピュータであるRaspberry Pi(以下、SBC)を起動し、ログインします。

   - 入力デバイスをSBCのUSBポートに接続します。
   - SBC用起動イメージが格納されたmicroSDカードを挿入します。
   - 電源を（USBまたはOpenCRのいずれかで）接続して、SBCをオンにします。
   - アカウント "ubuntu"、パスワード "turtlebot"でログインします。

     `SBCに電力を供給する前に、HDMIケーブルを接続する必要があります。接続しないと、SBCのHDMIポートが無効になります。`

1. 設定ファイルの修正
  
   修正対象ファイルを開く。


   *【SBC側で実施】*
   ```shell
   $ cd /etc/netplan
   $ sudo nano 50-cloud-init.yaml
   ```

   エディターが開いたら、`WIFI_SSID`と`WIFI_PASSWORD`をWi-FiSSIDとパスワードに置き換えます。(下イメージ緑文字部分)

   ![](images/seminar_no139/wifi_settings.png)

   修正後、ファイルを保存(`Ctrl + o` → `enter`)し、編集を終了（`Ctrl + x`）します。

   SBCを再起動します。

   *【SBCで実施】*

   ```shell
   $ sudo reboot
   ```

### ROSのネットワーク設定
SBCの再起動後、リモートPCのセットアップと同様([参照](#remotepc_wifi_settings))にSBCのIPアドレスを調べる（`wlan0`の箇所）。
   
*【SBCで実施】*
```shell
$ ip address show
```

エディタにて~/.bashrcを開きます。

*【SBCで実施】*
```shell
$ nano ~/.bashrc
```

下図のように、環境変数`ROS_MASTER_URI`と`ROS_HOSTNAME`のIPアドレスを、それぞれリモートPCのIPアドレスとSBCから取得したIPアドレスに変更します。

![](images/seminar_no139/wifi_ros_env.png)



ファイルを保存し（`Ctrl + o` → `enter`)、閉じる（`Ctrl + x`）。

さきほど修正した設定を反映するため、**~/.bashrcを編集したターミナルにて、** 以下を実行します。
(~/.bashrcの修正により、今後、新規にターミナルを開いた時は自動で設定が反映されます)

*【SBCで実施】*
```shell
$ source ~/.bashrc
```

## リモートPCからSBCへの接続方法

ワイヤレス構成が完了したら、デスクトップまたはノートパソコンからSSH経由でSBCに接続できます。

まずは、リモートPCとSBCでSSHをインストールする。

SBCにログイン後、以下を実行する。

*【SBCで実施】*
```shell
$ sudo apt-get install ssh
```

SBCでSSHを有効にする。

*【SBCで実施】*
```shell
$ sudo service ssh start
$ sudo ufw allow ssh
```
 
リモートPCから、SBCに接続する。
リモートPCにログインし、以下を入力する。

*【リモートPCで実施】*
```shell
$ ssh ubuntu@192.168.YY.YY   (@の後はSBCのIPアドレス)
 ubuntu@192.168.YY.YY\'s password: (パスワード "turtlebot" を入力)
 (初めて接続する場合、接続を継続するかを問われるので、yesを入力する)
```


SBCに接続できれば、設定は問題なくできています。

接続できない場合、
- sshで指定しているIPアドレス、アカウント、パスワードに誤りがないか
- リモートPCとSBCを同じwifiルーターに接続しているか
 
を確認してください。

以上で、事前準備は終了です。


 <!-- 

{% capture notice_03 %}

**注釈**: **公式 Raspbian Stretchとの変更点**
- [Raspbian Stretch with desktop](https://www.raspberrypi.org/software/)をベースにしています。 RaspbianはDebian Stretchをベースにしています。
- Wolfram、Mathematica、Minecraft Pi、Oracle Java SEなどの非フリーソフトウェアを削除しています。
- libreofficeを削除してイメージのサイズを小さくしています。
- raspi-configを使用してSSHおよびカメラ機能を有効化しています。
- パスワードを変更しています: **turtlebot**
- ROSおよびTurtleBot3のソフトウェアがインストール済みです。
  - [ROS Kinetic Kame](http://wiki.ros.org/kinetic)と依存パッケージ
  - [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node) Raspberry Pi Cameraの依存パッケージ
  - [hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)レーザ距離センサの依存パッケージ
  - [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)および、 [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
  のパッケージ
  - インストール済みのROSパッケージ (132パッケージ): actionlib, actionlib_msgs, angles, bond, bond_core, bondcpp, bondpy, camera_calibration_parsers, camera_info_manager, catkin, class_loader, cmake_modules, collada_parser, collada_urdf, common_msgs, compressed_image_transport, control_msgs, cpp_common, cv_bridge, diagnostic_aggregator, diagnostic_analysis, diagnostic_common_diagnostics, diagnostic_msgs, diagnostic_updater, diagnostics, dynamic_reconfigure, eigen_conversions, eigen_stl_containers, executive_smach, filters, gencpp, geneus, genlisp, genmsg, gennodejs, genpy, geometric_shapes, geometry, geometry_msgs, hls_lfcd_lds_driver, image_transport, joint_state_publisher, kdl_conversions, kdl_parser, message_filters, message_generation, message_runtime, mk, nav_msgs, nodelet, nodelet_core, nodelet_topic_tools, octomap (plain cmake), opencv3 (plain cmake), orocos_kdl (plain cmake), pluginlib, python_orocos_kdl (plain cmake), python_qt_binding, random_numbers, raspicam_node, resource_retriever, robot, robot_model, robot_state_publisher, ros, ros_base, ros_comm, ros_core, rosbag, rosbag_migration_rule, rosbag_storage, rosbash, rosboost_cfg, rosbuild, rosclean, rosconsole, rosconsole_bridge, roscpp, roscpp_core, roscpp_serialization, roscpp_traits, roscreate, rosgraph, rosgraph_msgs, roslang, roslaunch, roslib, roslint, roslisp, roslz4, rosmake, rosmaster, rosmsg, rosnode, rosout, rospack, rosparam, rospy, rosserial_msgs, rosserial_python, rosservice, rostest, rostime, rostopic, rosunit, roswtf, self_test, sensor_msgs, shape_msgs, smach, smach_msgs, smach_ros, smclib, std_msgs, std_srvs, stereo_msgs, tf, tf_conversions, tf2, tf2_kdl, tf2_msgs, tf2_py, tf2_ros, topic_tools, trajectory_msgs, turtlebot3_bringup, turtlebot3_msgs, urdf, urdf_parser_plugin, visualization_msgs, xacro, xmlrpcpp
{% endcapture %}
<div class="notice--info">{{ notice_03 | markdownify }}</div>

[appendix_raspi_cam]: /docs/en/platform/turtlebot3/appendix_raspi_cam/#raspberry-pi-camera
[pc_network_configuration]: /docs/en/platform/turtlebot3/pc_setup/#network-configuration
[network_configuration]: #5-network-configuration
[enable_ssh_server_in_raspberry_pi]: /docs/en/platform/turtlebot3/faq/#enable-ssh-server-in-raspberry-pi

-->