
<h1 align="center" style="border-bottom: none">
    <b>
        ⭐️ SLAM2REF ⭐️ <br>
    </b>
      Long-Term Mapping with 3D LiDAR and Reference Map Integration

</h1>

[//]: # (      Advancing Long-Term Mapping with 3D LiDAR and Reference Map Integration for Precise 6-DoF Trajectory Estimation and Map Extension  )

<p align="center">
Align and correct your LiDAR-based SLAM data with a reference map or a previous session.
</p>




[//]: # (#### [[ArXiv Paper &#40;better images&#41;]&#40;https://arxiv.org/abs/2408.15948&#41;] [[Paper]&#40;https://link.springer.com/article/10.1007/s41693-024-001&#41;] [[Data]&#40;https://mediatum.ub.tum.de/1743877&#41;])

<p align="center">
    <a href="https://arxiv.org/abs/2408.15948"><b>ArXiv Paper (better images)</b></a> •
    <a href="https://link.springer.com/article/10.1007/s41693-024-00126-w"><b>Paper (nicer to read)</b></a> •
    <a href="https://mediatum.ub.tum.de/1743877"><b>Data</b></a>
</p>

<p align="center">
  <a href="https://arxiv.org/abs/2408.15948">
    <img src="https://img.shields.io/badge/arXiv-2408.15948-%23B31C1B?style=flat" alt="arxiv">
  </a>
  <a href="https://youtu.be/5WgPRRijI4Y">
    <img src="https://img.shields.io/youtube/views/d_-ZYJhgGIk?label=YouTube&style=flat" alt="YouTube">
  </a>
  <img src="https://img.shields.io/badge/C++-Solutions-blue.svg?style=flat&logo=c%2B%2B" alt="C++">
  <img src="https://img.shields.io/github/license/MigVega/SLAM2REF" alt="License">

[//]: # (  <a href="https://github.com/MigVega/SLAM2REF">)

[//]: # (    <img src="https://img.shields.io/github/stars/MigVega/SLAM2REF" alt="GitHub Repo stars">)

[//]: # (  </a>)
  <a href="https://github.com/MigVega/SLAM2REF">
    <img src="https://img.shields.io/github/stars/MigVega/SLAM2REF.svg?style=flat&logo=github&colorB=deeppink&label=stars" alt="GitHub stars">
  </a>
  <a href="https://github.com/MigVega/SLAM2REF">
    <img src="https://img.shields.io/github/forks/MigVega/SLAM2REF" alt="GitHub forks">
  </a>
  <a href="https://github.com/MigVega/SLAM2REF">
    <img src="https://img.shields.io/github/issues/MigVega/SLAM2REF" alt="GitHub issues">
  </a>
</p>

[//]: # ([![arxiv]&#40;https://img.shields.io/badge/arXiv-2408.15948-%23B31C1B?style=flat&#41;]&#40;https://arxiv.org/abs/2408.15948&#41;)

[//]: # ([![YouTube]&#40;https://img.shields.io/youtube/views/d_-ZYJhgGIk?label=YouTube&style=flat&#41;]&#40;https://youtu.be/5WgPRRijI4Y&#41;)

[//]: # (![C++]&#40;https://img.shields.io/badge/C++-Solutions-blue.svg?style=flat&logo=c%2B%2B&#41;)

[//]: # (![License]&#40;https://img.shields.io/github/license/MigVega/SLAM2REF&#41;)

[//]: # ([![GitHub Repo stars]&#40;https://img.shields.io/github/stars/MigVega/SLAM2REF&#41;]&#40;https://github.com/MigVega/SLAM2REF&#41;)

[//]: # (<a href="https://github.com/MigVega/SLAM2REF"><img src="https://img.shields.io/github/stars/MigVega/SLAM2REF.svg?style=flat&logo=github&colorB=deeppink&label=stars"></a>)

[//]: # ([![GitHub forks]&#40;https://img.shields.io/github/forks/MigVega/SLAM2REF&#41;]&#40;https://github.com/MigVega/SLAM2REF&#41;)

[//]: # ([![GitHub issues]&#40;https://img.shields.io/github/issues/MigVega/SLAM2REF&#41;]&#40;https://github.com/MigVega/SLAM2REF&#41;)
<!-- TO ADD -->
<!-- ![GitHub Workflow Status (with event)](https://img.shields.io/github/actions/workflow/status/rayguan97/crossloc3d/.github%2Fworkflows%2Fpython-package-conda.yml)-->
<!-- ![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/dataset/slam2ref)(https://paperswithcode.com/sota)-->
<!-- ![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)) -->




## What is SLAM2REF?
SLAM2REF uses pose-graph multi-Session anchoring to align your LiDAR data with a reference map or with another session, allowing precise 6-DoF pose retrieval and map extension.
- This project is an extension of [LT-SLAM](https://github.com/gisbi-kim/lt-mapper/tree/main), which implements a custom GTSAM factor for anchoring (see BetweenFactorWithAnchoring.h). However, this project is completely ROS-independent. This is also an extension of the [BIM-SLAM](http://www.iaarc.org/publications/2023_proceedings_of_the_40th_isarc_chennai_india/bim_slam-integrating_bim_models_in_multi_session_slam_for_lifelong_mapping_using_3d_lidar.html) project, for which a [video](https://youtu.be/5WgPRRijI4Y) explanation is available.
- Moreover, we have implemented a novel Indoor Scan Context Descriptor for fast place recognition.
- Also, a novel YawGICP algorithm for robust point cloud registration with varying mostly yaw angles.
- SLAM2REF additionally allows the retrieval of 6-DoF poses with an accuracy of up to 3 cm given an accurate TLS point cloud as a reference map (this map should be accurate, at least regarding the position of permanent elements such as walls and columns).

The following image presents a very brief overview of how the method works.
<p align="center"><img src="doc/imgs/Gtihub_overview__.png" alt="SLAM2REF Github - Overview" width="80%" /></p>


    
## How to run the code?

For Building, add this flag to use only five threads `-j 5`; otherwise, the project might exit before building.

- After successfully building the project, all the input paths (to the query and central sessions, for example) and parameters are given in the file `config/params.yaml`.

- Then, simply running the code (with the play bottom) should start the execution.
    in the console, the following should be visible:
  
    ```bash
    ----> Slam2ref starts.
    ```

- Once the program has been successfully finalized, you should see the following:
  
    ```bash
    ----> Slam2ref done.
    ```


## Stay Up-to-Date / Support
Start the repo!
<p align="center"><img src="doc/imgs/github_start_only.gif" alt="SLAM2REF Github - how to star the repo" width="50%" /></p>


## License
For academic usage, the code is released under the [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html). 

For any commercial purpose, please contact the author.

## Citation
If you use this work or our data in your research, please include the following citations (these BibTeX entries are the best versions you will likely find ✔️).

**Paper & Data:**
The data consists of the BIM Model of [ConSLAM](https://github.com/mac137/ConSLAM) and Ground Truth poses.

```BibTeX
@article{SLAM2REF:vega2024:paper,
	title        = {{SLAM2REF}: advancing long-term mapping with {3D} {LiDAR} and reference map integration for precise 6-{DoF} trajectory estimation and map extension},
	author       = {Vega-Torres, Miguel A. and Braun, Alexander and Borrmann, André},
	year         = 2024,
	month        = 7,
	journal      = {Construction Robotics},
	publisher    = {Springer},
	volume       = 8,
	number       = 2,
	pages        = 13,
	doi          = {10.1007/s41693-024-00126-w},
	url          = {https://link.springer.com/article/10.1007/s41693-024-00126-w},
	notes        = {link to code: https://github.com/MigVega/SLAM2REF/. Link to data: https://mediatum.ub.tum.de/1743877},
	keywords     = {LiDAR; Multi-Session SLAM; Pose-Graph Optimization; Loop Closure; Long-term Mapping; Change Detection; {BIM} Update; {3D} Indoor Localization and Mapping},
	language     = {en}
}

@inproceedings{BIM_SLAM:vega:2023,
	author = {Vega-Torres, Miguel A. and Braun, A. and Borrmann, A.},
        title = {{BIM-SLAM: Integrating BIM Models in Multi-session SLAM for Lifelong Mapping using 3D LiDAR}},
	booktitle = {Proc. of the 40th International Symposium on Automation and Robotics in Construction (ISARC 2023)},
	year = {2023},
	month = {07},
        isbn = {978-0-6458322-0-4},
	doi = {10.22260/ISARC2023/0070},
	keywords = {BIM; LiDAR; SLAM },
	url = {http://www.iaarc.org/publications/2023_proceedings_of_the_40th_isarc_chennai_india/bim_slam-integrating_bim_models_in_multi_session_slam_for_lifelong_mapping_using_3d_lidar.html},
}

@misc{SLAM2REF:vega2024:data,
	title        = {{ConSLAM} {BIM} and {GT} Poses},
	author       = {Vega-Torres, Miguel A. and Braun, Alexander and Borrmann, André},
	year         = 2024,
	month        = 6,
	publisher    = {Technical University of Munich},
	doi          = {10.14459/2024MP1743877},
	url          = {https://mediatum.ub.tum.de/1743877},
	type         = {Dataset},
	abstract     = {The ConSLAM BIM and GT Poses dataset comprehends the 3D building information model (in IFC and Revit formats), manually elaborated based on the terrestrial laser scanner of the sequence 2 of ConSLAM, and the refined ground truth (GT) poses (in TUM format) of the sessions 2, 3, 4 and 5 of the open-access Con{SLAM} dataset. This dataset can be found here: https://github.com/mac137/ConSLAM},
	keywords     = {LiDAR; Multi-Session SLAM; Pose-Graph Optimization; Loop Closure; Long-term Mapping; Change Detection; {BIM} Update; {3D} Indoor Localization and Mapping},
	language     = {en}
}
```
**Code:** To be added.

## Acknowledgements
This is an extension of [LT-SLAM](https://github.com/gisbi-kim/lt-mapper/tree/main/ltslam) (2022), whose author is Giseop Kim.
