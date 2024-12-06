pitch=("0" "-10" "-20" "-30" "-40" "-50")
replay=("train" "validate" "test")
output=("train" "train" "test")
lidar="livox"
height="6"
# 
# replay base for bird-eyed view (BEV) preview
# 
root="/media/hdd1/opv2v/"
outp="/media/hdd1/opv2v/opencda_dump"
for ((i=0; i<${#replay[@]}; i++))
do
    python opencda_replay.py -t logreplay_bev -r ${root}/${replay[$i]}_${lidar}_${height}_${p} -o ${outp}/${output[$i]}_bev_-90
done
# # 
# # replay original for base
# # 
# root="/media/hdd1/opv2v/"
# outp="/media/hdd1/opv2v/opencda_dump"
# for p in ${pitch[*]}
# do
#     for ((i=0; i<${#replay[@]}; i++))
#     do
#         python opencda_replay.py -t logreplay -r ${root}/${replay[$i]}_${lidar}_${height}_${p} -o ${outp}/${output[$i]}_${lidar}_${height}_${p}
#     done
# done
# # 
# # replay base for variations
# # 
# root="/media/hdd1/opv2v/opencda_dump"
# outp="/media/hdd1/opv2v/opencda_dump"
# for p in ${pitch[*]}
# do
#     for ((i=0; i<${#replay[@]}; i++))
#     do
#         python opencda_replay.py -t logreplay_metric -r ${root}/${replay[$i]}_${lidar}_${height}_${p} -o ${outp}/metric_${output[$i]}_${lidar}_${height}_${p}
#     done
# done
# # 
# # compute metrics for each scenes in test set
# # 
# outp="/media/hdd1/opv2v/opencda_dump"
# for p in ${pitch[*]}
# do
#     python ComputeDNUC.py --dir ${outp}/metric_test_${lidar}_${height}_${p}
# done