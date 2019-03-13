# Start to Detect Black Line

    $ rosrun scan_black detect_black.py

## Publish Topic
    /scan_black/scaninfo

## Subscriber Topic 
    /scan_black/middleY
    /scan_black/range
    /scan_black/weight
    /scan_black/threshold
    /scan_black/scamMum

## scaninfo msg
    int     dis         -> distance error
    int32[] scanstate   -> detect black line state

## ※ Parameter Setting ##
    * middleY    -> 可視框垂直移動 
    * range      -> 可視框範圍
    * weight     -> 檢測黑線 輸入百分比 
    * threshold  -> 亮度閾值
    * scanNum    -> 感測器數量

