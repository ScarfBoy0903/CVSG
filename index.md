###**[論文筆記] Real-Time Monocular Pose Estimation of 3D Objects using Temporally Consistent Local Color Histograms**

這是小弟第一篇論文筆記，將來應該會每週更新一篇論文筆記，自己還很菜，文中有些基礎部分知識還沒理解的部分會略過，有機會的話會再補足，若內文有錯誤還請各位多多包涵不吝指正。
<br><br>

####**背景介紹：**
實踐中通常會希望單台相機而不是設置多台相機使其硬體要求以及校準拼接的成本達到最低，針對以往的影像姿態及目標辨識中幾個較難克服的議題：**occlusion(目標受到部分阻擋物)**、**cluttered backgrounds(複雜場景使目標混淆)**、**large scale change(畫面變化快速)**等問題，本文中介紹了一個抓取輪廓後採取顏色分布作為前景以及背景機率的判斷方法來達到pose的tracking以及detection並在上述問題中達到了較為robust的效果並且在計算上耗費的時間較短。

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

此外, 真實世界在投影至相機的過程中會有distortion的狀況, 矩陣 $K$ 的作用為一將其位置做**Calibration(修正)**，$K$ 的element稱為**intrinsic parameters**，其係數通常取決於distortion的種類，而distortion通常取決於相機本身的特性，而 $K$ 內部組成因其過程較為複雜在此先省略。
![这里写图片描述](https://img-blog.csdn.net/2018091322024984?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

![这里写图片描述](https://img-blog.csdn.net/20180913223151832?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
$\pi (\boldsymbol X)={(X_{n}/Z_{n}, Y_{n}/Z_{n}, 1)^T \in \mathbb{R^2}}$ 

$\tilde X$ 經過 $K$ 與 $T$ 之後的結果為相機座標下且已經過修正的四維位置向量，$\pi(.)$ 為表示將三維空間投影至二維空間並藉由平面與座標基準點的比例來做scaling修正。

以上內容為參考[Robust Real-Time Visual Tracking using Pixel-Wise Posteriors](http://www.robots.ox.ac.uk/~cbibby/pubs/ECCV08.pdf)

<br>
### **Segmentation：**
![这里写图片描述](https://img-blog.csdn.net/20180913231228406?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
在pose tracking之前需要建立**contour(目標投影在二維影像的輪廓)**來作為區分**foreground(前景)**以及**background(背景)**的依據，其核心思想為透過局部區域偵測不同$X_{n}$的**local color histogram(圖像中不同偵測地區的顏色分布)**，透過[Robust Real-Time Visual Tracking using Pixel-Wise Posteriors](https://ieeexplore.ieee.org/document/4636741/)提及的方法來抓住高度重疊的部分偵測能量的方法來抓取輪廓，其參考論文中，**region-based**相較於參考整個區域，對於初始化曲線位置的判斷較為robust，且具有抗噪特性，對於噪聲較不敏感，每個輪廓點在自己的局部區域內能夠最小化局部能量，進而忽略其他较遠的地方可能出现的灰度不均匀狀況。

<br>
### **Pose Tracking：**
方法為參考[PWP3D- Real-time Segmentation and Tracking of 3D Objects](http://www.robots.ox.ac.uk/~victor/pdfs/prisacariu_reid_ijcv2012_draft.pdf)及作者過去的發表[Robust Real-Time Visual Tracking using Pixel-Wise Posteriors](https://ieeexplore.ieee.org/document/4636741/)。

在平面上目標的**contour**建立起來後，目標在整個相機座標二維成像下的樣貌已大抵確立，而整個**pose tracking**的過程是利用「每個位置上顏色分布的條件下是否為前景的機率」做為目標物的依據來進行追蹤，其3D形狀的依據為被判斷為**contour**內外與輪廓的遠近，即點至輪廓切線的垂直距離，來作為距離成像遠近的估計量值。
![这里写图片描述](https://img-blog.csdn.net/20180914020457246?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

其計算距離的定義為Level-Set Pose Embedding：
![这里写图片描述](https://img-blog.csdn.net/20180914021048879?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
如上述，落在背景距離為正，落在前景距離為負。
<br><br><br>
除了輪廓與形狀之外，亦須判斷其估計的合理性：
${P(\boldsymbol\Phi | \boldsymbol I) = \displaystyle \prod_{x \in \Omega} (H_{e}(\boldsymbol\Phi(\boldsymbol x))P_{f}+(1-H_{e}(\boldsymbol\Phi(\boldsymbol x)))P_{b})}$, 
 其中，$H_{e}(\boldsymbol\Phi(\boldsymbol x))P_{f}+(1-H_{e}(\boldsymbol\Phi(\boldsymbol x))P_{b}$ 代表的是在現在偵測的影像條件下該位置對應在距離輪廓為 $\boldsymbol\Phi$的機率，而每個位置點的機率為獨立，則其所有位置點的交集，即為現在影像是否如預測的機率。

- ${H_{e}(.)}$ **Heaviside step function**, 一般可看作為一普通step function或是sigmoid，而參考論文中定義為：$He(x) = (1/\pi)(-atan(b·\boldsymbol x)+\pi/2)$, $\boldsymbol x$ 為前景時$~ H_{e}\boldsymbol(\Phi)~為1，反之則為~0$。

- ${P_f,~P_b}$ 為在該像素$x_n$中對應到顏色條件下是否為前景亦或是後景的機率，可記作$P(M_f | \boldsymbol y),~P(M_b| \boldsymbol y)~,~$其中$M_f，M_b$ 為前景及後景的事件, $\boldsymbol y={I_{c}(\boldsymbol x)}$ , $\boldsymbol y$ 代表的是color pixel，成像上的一個顏色點。

而上述的機率可以作為一測量依據來判斷現在的**contour**是否為真正目標的輪廓，文中定義了一**能量**作為一個指標：
以取預測機率的對數，以量級作為其判斷依據：
$\boldsymbol E(\boldsymbol\Phi) = -log(P(\boldsymbol\Phi | \boldsymbol I)) = -\displaystyle \sum_{x \in \Omega}(H_{e}(\boldsymbol\Phi(\boldsymbol x))P_{f} + (1-H_{e}(\boldsymbol\Phi(\boldsymbol x)))P_{b})$
設置一$threshold~t$，若$E/| \Omega | > t $, 表示其各點$P_f$ 以及 $P_b$ 所形成的交集機率過小，則判定該pose tracking已經lost，此時前一刻的frame已不可信賴，需要使用不依賴記憶性的pose detection。

另外，$P(M_f | \boldsymbol y),~P(M_b| \boldsymbol y)$ 等條件機率的求得方法，內文使用的是Bayes Theorem，貝氏定理通常用於求解難以求得的條件機率，但若欲求條件與事件交換時可求解的已知機率與沒有條件考量的機率可得的狀況作為依據，可以現有經驗去推估其結果。
![这里写图片描述](https://img-blog.csdn.net/20180914035058343?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
其中 $\displaystyle\sum_{x \in \Omega}(H_{e}(\boldsymbol\Phi(\boldsymbol x))$ 所代表的物理意義為前景像素的數量，原因為在**contour**內與之的距離$\Phi(\boldsymbol x) > 0$，而作為階函數的輸入則為1，反之則為0。

文中介紹的tracking所需要上一刻frame，則在於求取 $likehood ~~P(M_f| \boldsymbol y)及P(M_b| \boldsymbol y)$ ，使用的是一假設Temporal Consistency條件下的近似疊代式：

$P^t(M_i| \boldsymbol y) = \alpha _i P^t(M_i| \boldsymbol y) + (1 - \alpha _i)P^{t-1}(M_i| \boldsymbol y)$ ,$~~i \in {f,b}$ ,   $\alpha _i$ 為一自訂參數(learning rate)

藉著疊代上一刻結果的方法來預測下一刻，自己嘗試對其物理意義作一些直觀的猜測，應可看作是linear interpolation近似的結果，從時間一致性字面上來看，在時間間隔很短的狀況下下一刻與上一刻之間的likehood與實際上下一刻的likehood非常相近。
![这里写图片描述](https://img-blog.csdn.net/20180914044107931?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0Rlbm5pc19MZWVf/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
<br><br><br><br>
以上為自己近期該篇論文的心得記錄，剩下的部分, optimization algorithm以及pose detection自己的基礎尚且不夠沒能看懂看完，之後有機會會再補上。
