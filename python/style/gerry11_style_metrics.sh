# Base  ClassifierFree1  Control  Decoupled  Finetuned  Guidance
fidelity --gpu 0 --fid --prc --input1 ./results/gerry11_all_networks/Base/ --input2 ./data/gml_images_2 > ./results/gerry11_all_networks/base_metrics.txt
fidelity --gpu 0 --fid --prc --input1 ./results/gerry11_all_networks/ClassifierFree1/ --input2 ./data/gml_images_2 > ./results/gerry11_all_networks/ClassifierFree1_metrics.txt
fidelity --gpu 0 --fid --prc --input1 ./results/gerry11_all_networks/Control/ --input2 ./data/gml_images_2 > ./results/gerry11_all_networks/Control_metrics.txt
fidelity --gpu 0 --fid --prc --input1 ./results/gerry11_all_networks/Decoupled/ --input2 ./data/gml_images_2 > ./results/gerry11_all_networks/Decoupled_metrics.txt
fidelity --gpu 0 --fid --prc --input1 ./results/gerry11_all_networks/Finetuned/ --input2 ./data/gml_images_2 > ./results/gerry11_all_networks/Finetuned_metrics.txt
fidelity --gpu 0 --fid --prc --input1 ./results/gerry11_all_networks/Guidance/ --input2 ./data/gml_images_2 > ./results/gerry11_all_networks/Guidance_metrics.txt