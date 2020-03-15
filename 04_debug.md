# 1. ROSでデバッグ (Visual studio code & ROS & python) v1.2
・ここではvscode(visual studio code)を使い、ROS上でデバッグする方法について記載する。  
・更新日 2019年08月26(火) 20:00　by gogo5nta
## 1. VSCODEの入手
・以下のサイトからvscodeを入手。ubuntuなので.debを選択  
  [Visual Studio Code](https://code.visualstudio.com/)
  ![190818-01](https://user-images.githubusercontent.com/30023363/63224816-80ea3080-c204-11e9-82be-349dacbd4514.jpg)

## 2. vscodeのインストール
・ダウンロードしたファイル(例:/Download/code_xxx.deb)をマウス右ボタンクリック > ソフトウェアのインストールで開くを選択
 ![190818-02](https://user-images.githubusercontent.com/30023363/63224982-c6a7f880-c206-11e9-8baf-aaed7acc92e4.jpg)  
  または、以下のコマンドでもOK
  ```
  $ cd ~/Download
  $ sudo dpkg -i code_*.deb
  ```
## 3. vscodeの設定
### 3.1 vscodeの起動
・コンピュータの検索でcodeと入力　または、ターミナル上でcodeと入力
 ```
 $ code  
 ```
![190818-03](https://user-images.githubusercontent.com/30023363/63225002-f7882d80-c206-11e9-87b6-018d44338767.jpg)  
### 3.2 vscodeの設定(初回)
・拡張機能をインストールする。起動したら、左側の５個のアイコンの中で、一番したのアイコンを押し、以下の拡張機能をインストール  
・C/C++　　　　　　　　//C++本体。★必須★  
・C++ Intellisense　 //コード補完★必須★  
・C/C++ Snippets　　//コード一式を登録する機能。入れなくてもOK  
・ctags　　　　　　　//関数ジャンプ。F1でリスト作成。入れなくてもOK  
・Python　　　　　　　//python本体★必須★  
・Python for VSCode　//python用。入れたほうがいい？  
・ROS　　　　　　　　　//ROS用★必須★  
・ROS snippests　　　//ROS用。入れたほうがいい  
![190818-04](https://user-images.githubusercontent.com/30023363/63225143-42ef0b80-c208-11e9-9ab1-3d1527406f47.jpg)  

### 3.3 vscodeの日本語化(必要な方)
・拡張機能で、Japanese Language Pack for VS Codeをインストール  
![190819-01](https://user-images.githubusercontent.com/30023363/63270815-d5a8ac80-c2d3-11e9-834f-1e1aa675b55b.jpg)  
・再起動を聞かれたらYesを選択  
![190819-02](https://user-images.githubusercontent.com/30023363/63270867-e9eca980-c2d3-11e9-9ba5-a16c2848b74b.jpg)  
・参考  [Visual Studio Codeを日本語化する方法](https://qiita.com/HiroCh/items/481adfa969dbe689f566)  

## 4. ROSで動かす設定
### 4.1 フォルダの選択　　
・gitで落としてきたburger_warのフォルダを選択。 例
```
cd ~/catkin_ws/src
git clone https://github.com/gogo5nta/burger_war
```
・まずは、左側アイコンの一番上を押し、Open Folderを選択　　
  ~/catkin_ws/src/burger_warを指定  
![190818-05](https://user-images.githubusercontent.com/30023363/63225778-0c68bf00-c20f-11e9-86fd-b803b1298e42.jpg)
### 4.2 c_cpp設定(初回、cpp使用時。pythonのみなら飛ばしてOK)
・.vscodeフォルダ内のc_cpp>proprties.jsonを選択し、以下を追加  
・[VSCodeでROS関連の補完（C++)](https://www.asrobot.me/entry/2018/12/21/015433)
```
     "includePath": [
        "${workspaceFolder}/**",
        "/opt/ros/kinetic/include/**",
        "/usr/include/**"
        ・
        ・
```
![190818-06](https://user-images.githubusercontent.com/30023363/63225962-9580f580-c211-11e9-9049-5317cc05402b.jpg)　　

### 4.3 aliasの設定と、terminatorのインストール
・複数ターミナルを起動するため、terminatorを事前にインストールする。 
```
$ sudo apt-get install terminator
```
・また、短いコマンドでシミュレータ等を起動するため、~/.bashrcファイルをエディタ等で開き、  
  設定を追加後、source ~/.bashrcで読み込ませておく  
```
# ROS全般コマンド  
alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'
# robocon2019用コマンド  
alias bw='cd ~/catkin_ws/src/burger_war'
alias sim='cd ~/catkin_ws/src/burger_war && bash scripts/sim_with_judge.sh'
alias start='cd ~/catkin_ws/src/burger_war && bash scripts/start.sh'
# robot位置がリセット。↑のstartで再スタート  
alias reset='rosservice call /gazebo/reset_simulation "{}"' 
```

### 4.4 python用にlaunchファイルの修正(vscodeでデバッグする場合)
・buger_war/launch/your_burger.launchファイル内のrandomRun.pyをvscodeでデバッグ(起動)するためコメントアウト
```
    <!-- sample program node -->
    <!-- comment out if you debug python with vscode -->
    <!--
    <node pkg="burger_war" type="randomRun.py" name="randomRun" output="screen"/>
    -->
    <!-- END sample program node -->
```
![190818-07](https://user-images.githubusercontent.com/30023363/63226511-3f15b600-c215-11e9-9215-6e6cde6d3f02.jpg)　　

### 4.5 python 2.7でデバッグするためのおまじない
・Ctrl+Shift+P（同時押し）で"コマンドパレット"というものが開くのでpython select interpreterを検索  
・python2.7xxxを選択。　　
![190818-09](https://user-images.githubusercontent.com/30023363/63227043-2a3d2080-c21d-11e9-82f3-7780410d44a7.jpg)  
・defaultではpython3が選択されているため、vscodeでデバッグすると以下のエラーが発生。  
  解決するまでに半日かかった(T_T)  
  ```
    from . message import Message, SerializationError, DeserializationError, MessageException, struct_I
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/genpy/message.py", line 44, in <module>
    import yaml
ImportError: No module named 'yaml'  
  ```
・上記切り替えは、setting.json内の以下を切り替えると同じ
   ```
   "python.pythonPath": "/usr/bin/python"   
   ```
![190819-04](https://user-images.githubusercontent.com/30023363/63271769-c75b9000-c2d5-11e9-960e-da35e48b0caa.jpg)　　

### 4.6 /red_bot/のNAMESPACEを事前に定義
・vscode上でrandomRun.pyを起動すると、NAMESPACE(red_bot)が与えられないので、　　
　launch.jsonを開き、"env"で設定する。  
  参考  [VSCode 用の launch.json でデバッグコマンドに環境変数 / 引数 / フラグを渡す設定](https://qiita.com/YumaInaura/items/72c1c68e22c71f37cdd3)
 ```
    "configurations": [
        
        {
            "name": "Python: Current File",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal",
            "env": {
                "ROS_NAMESPACE": "red_bot"
            }
        }
    ] 
 ```
![190819-05](https://user-images.githubusercontent.com/30023363/63272718-86fd1180-c2d7-11e9-8363-5a4a6d363248.jpg)

### 4.7　pythonでデバッグ
・terminatorを起動し、Ctrl+Shift+O (or Ctrl + Shift + e)でターミナルを２つに分割。<BR>
  ターミナル1: sim   (aliasを使用)<BR>
  ターミナル2: start (aliasを使用。gazebo起動後に実施)<BR>
  ![190818-10](https://user-images.githubusercontent.com/30023363/63227088-e1d23280-c21d-11e9-84a2-d83a7f8b1a16.jpg)  
・vscodeでrandamRun.pyを開き、適当な行にブレークをはる  
![190818-07](https://user-images.githubusercontent.com/30023363/63227019-e4805800-c21c-11e9-8c4c-2f56d4b48e83.jpg)　　
・左側アイコンの虫眼鏡をおし、DEBUG > python:Current Fileを選択
![190818-08](https://user-images.githubusercontent.com/30023363/63227022-f7932800-c21c-11e9-99d2-dd8d24658d7b.jpg)　　

### 4.8 python上のcws(カレントワークディレクトリ)
・vscode上でデバッグの場合、 cwsは  
```
~/catkin_ws/src/buger_war
```
　※vscodeで最初にフォルダを開く際、~/catkin_ws/src/buger_warを指定のと同じcwdに  
   
・一方、roslaunch(your_burger.launchや、setup.launch)で起動する場合、cwsは
```
~/.ros
```
　※python上で画像読込の場合、cwdからのpathを指定する事  
   
・VSCodeやroslaunch上など環境依存せず取得する場合、以下が参考  
  [[Python] スクリプト実行ディレクトリを絶対パスで取得](https://qiita.com/nagamee/items/b7d1b02074293fdfdfff)  
  例:pythonを実行したフォルダと同じフォルダにimg.jpgがある場合
```
img = cv2.imread(os.path.dirname(__file__) + "/img.jpg", 1)
```
 
## 5. vscodeのgitでソース管理
### 5.1 事前設定
・以下のgit全体設定を実施する。　　
```
$ git config --global user.name 'username'
$ git config --global user.email 'username@example.com'
```
### 5.2 commit
・左アイコン真ん中のツリーアイコンを押し、source管理を実施。コミットはチェックボタンを押すだけ
![190818-11](https://user-images.githubusercontent.com/30023363/63227569-baca2f80-c222-11e9-8768-9c9f1c95c560.jpg)  
 ・サーバー上へpushする場合、…のアイコンでpushを選択　　
![190818-12](https://user-images.githubusercontent.com/30023363/63227592-f36a0900-c222-11e9-84f1-8fa185fb288b.jpg)
 ・参考サイト  
 　 [VSCodeでのGitの基本操作まとめ](https://qiita.com/y-tsutsu/items/2ba96b16b220fb5913be)
 
 

