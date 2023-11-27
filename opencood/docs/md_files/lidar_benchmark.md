## Benchmark: 3D LiDAR  Detection

---
### Results on OPV2V dataset (AP@0.7 for no-compression/ compression)

|                    | Backbone   | Fusion Strategy  | Bandwidth (Megabit), <br/> before/after compression| Default Towns    |Culver City| Download |
|--------------------| --------   | ---------------  | ---------------                | -------------    |-----------| -------- |
| Naive Late         | PointPillar        | Late      |    **0.024**/**0.024** |   0.781/0.781        | 0.668/0.668         |    [url](https://drive.google.com/file/d/1WTKooW6k0exLqoIE5Czqy6ptycYlgKZz/view?usp=sharing)   |
| [Cooper](https://arxiv.org/abs/1905.05265)       | PointPillar        | Early  |   7.68/7.68   | 0.800/x         | 0.696/x       | [url](https://drive.google.com/file/d/1N1p6syxGSKD18ELgtBQoSuUzR8tX1JeE/view?usp=sharing)     | 
| [Attentive Fusion](https://arxiv.org/abs/2109.07644)         | PointPillar        | Intermediate  | 126.8/1.98   | **0.815**/**0.810**       | **0.735**/**0.731**        | [url](https://drive.google.com/file/d/1u4w13SDzdGq6Irh2PHxT-qIlNXRT3z6Z/view?usp=sharing)     | 
| [F-Cooper](https://arxiv.org/abs/1909.06459)         | PointPillar        | Intermediate  | 72.08/1.12    | 0.790/0.788     | 0.728/0.726        | [url](https://drive.google.com/file/d/1CjXu9Y2ZTzJA6Oo3hnqFhbTqBVKq3mQb/view?usp=sharing)     | 
| Naive Late         | VoxelNet        | Late  | **0.024**/**0.024**    | 0.738/0.738          | 0.588/0.588        | [url]()    |
| Cooper    | VoxelNet        | Early   |   7.68/7.68  | 0.758/x        | 0.677/x        | [url](https://drive.google.com/file/d/14WD7iLLyyCJJ3lApbYYdr5KOUM1ACnve/view?usp=sharing)     | 
| Attentive Fusion        | VoxelNet        | Intermediate |   576.71/1.12   | **0.864**/**0.852**        | **0.775**/**0.746**       | [url](https://drive.google.com/file/d/16q8CfcB8dS4EVhJMvvEfn0gM2ynxZB3E/view?usp=sharing)      | 
| Naive Late         | SECOND        | Late |  **0.024**/**0.024**    |  0.775/0.775        |0.682/0.682        | [url](https://drive.google.com/file/d/1VG_FKe1mKagPVGXH7UGHpyaM5q3cxtD8/view?usp=sharing)      |
| Cooper    | SECOND        | Early  |   7.68/7.68   |  0.813/x       |  0.738/x     | [url](https://drive.google.com/file/d/1Z9io1VNcU-urcRW8l0ogWCTVCB53mw4N/view?usp=sharing)     | 
| Attentive         | SECOND        | Intermediate |  63.4/0.99     |   **0.826**/**0.783**     | **0.760**/**0.760**    | [url](https://drive.google.com/file/d/1zEB8EyZ0X-WQykHFOM0pVwI8jXunRz1Z/view?usp=sharing)      | 
| Naive Late         | PIXOR        | Late |    **0.024**/**0.024** |    0.578/0.578       |  0.360/0.360      | [url]()      |
| Cooper    | PIXOR        | Early |   7.68/7.68    |   0.678/x      | **0.558**/x      | [url](https://drive.google.com/file/d/1ZDLjtizZCuV6D92LloEPKRIw-LqxfE1j/view?usp=sharing)     | 
| Attentive         | PIXOR        | Intermediate  |   313.75/1.22  |  **0.687**/**0.612**      | 0.546/**0.492**       | [url]()      |


**Note**: 
* We suggest using **PointPillar** as the backbone when you are creating your method and try to compare with
our benchmark, as we implement most of the SOTA methods with this backbone only.
* We assume the transimssion rate is 27Mbp/s. Considering the frequency of LiDAR is 10Hz, the 
bandwidth requirement should be less than **2.7Mbp** to avoid severe delay. 
* A 'x' in the benchmark table represents the bandwidth requirement is too large, which 
can not be considered to employ in practice.