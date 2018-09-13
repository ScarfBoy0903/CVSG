###[論文筆記] Real-Time Monocular Pose Estimation of 3D Objects using Temporally Consistent Local Color Histograms

這是小弟第一篇論文筆記，將來應該會每週更新一篇論文筆記，自己還很菜，文中有些基礎部分知識還沒理解的部分會略過，有機會的話會再補足，若內文有錯誤還請各位多多包涵不吝指正。
<br><br>

####**背景介紹：**
實踐中通常會希望單台相機而不是設置多台相機使其硬體要求以及校準拼接的成本達到最低，針對以往的影像姿態及目標辨識中幾個較難克服的議題：**occlusion(目標受到部分阻擋物)**、**cluttered backgrounds(複雜場景使目標混淆)**、**large scale change(畫面變化快速)**等問題，本文中介紹了一個抓取輪廓後採取顏色分布作為前景以及背景機率的判斷方法來達到pose的tracking以及detection並在上述問題中達到了較為robust的效果。

步驟大致上可以分為四個步驟：**Projection**、**Segmentation**、**Pose Tracking**、**Pose Detection**
<br><br>
### **Projection：**
首先會在畫面中尋找各個**feature point** 然後再將各點連線形成數個**triangular mesh(三角近似平面網)**來描述目標在現實中的形狀。

投影在相機座標下二維成像畫面座標為：$x = \pi(KT(\tilde X))\in\mathbb{R^2} $
$\boldsymbol{X}:={(X_{n}, Y_{n}, Z_{n})^T \in \mathbb{R^3}},~n = 1. . . N$ 

- $\boldsymbol{X}$為三維世界座標下的代表第n個三角近似平面的位置點
- ${N}$為三角近似平面的數量
- ${Z_{n}}$為與座標基準點的距離

$\boldsymbol{\tilde X}={(X, Y, Z, 1)^T \in \mathbb{R^3}}$ , 為 $\boldsymbol{X}$ 座標的齊性表示法，多一個維度來表示的目的為令矩陣能達到相乘向量後能夠加上另一常數向量，其物理意義為空間中兩點平移量。

在三維空間中需要以不同的位置上看到的位置是相對的，在將三維座標投影至二維成像平面之前需要使座標統一，撇除distortion的問題，三維空間中不同基準點產生的座標若互相轉換時，需要考量的物理效應為**Rotation(旋轉)**、**Translation(平移)**，此轉換過程為線性轉換，可用矩陣來表示其過程，即notation中的 ${T}$ ，過程如下圖示：
![这里写图片描述](https://img-blog.csdn.net/20180913212633472?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
$T = 
\begin{bmatrix} 
R_{3 \times 3} & T_{r3 \times 1} \\
O_{1 \times 3} & 1  
\end{bmatrix}\_{4 \times 4}$$~~~~~R$ 代表Rotation, $T_{r}$  代表Translation的block, T矩陣的element稱為**extrinsic parameters**。    

此外, 真實世界在投影至相機的過程中會有distortion的狀況, 矩陣 $K$ 的作用為一將其位置做**Calibration(修正)**，$K$ 的element稱為**intrinsic parameters**，因其過程較為複雜在此先省略。
![这里写图片描述](https://img-blog.csdn.net/2018091322024984?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

![这里写图片描述](https://img-blog.csdn.net/20180913223151832?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
$\pi (\boldsymbol X)={(X_{n}/Z_{n}, Y_{n}/Z_{n}, 1)^T \in \mathbb{R^2}}$ 

$\tilde X$ 經過 $K$ 與 $T$ 之後的結果為相機座標下且已經過修正的四維位置向量，$\pi(.)$ 為表示將三維空間投影至二維空間並藉由平面與座標基準點的比例來做scaling修正。

以上內容為參考[Robust Real-Time Visual Tracking using Pixel-Wise Posteriors](http://www.robots.ox.ac.uk/~cbibby/pubs/ECCV08.pdf)
