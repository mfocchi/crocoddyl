
<p align="center">
  <img src="./doc/images/crocoddyl_logo.png" width="800" alt="Crocoddyl Logo" align="center"/>
</p>

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/>

## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building from source

```bash
dock-root
cd CROCODDYL_DIR
mkdir build
cd build
cmake ..
make install
```

In addition to Locosim you will need to configure your environment variables, e.g.:

```bash
export PKG_CONFIG_PATH=/usr/local/lib/:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python3/dist-packages:$PYTHONPATH
```

this will override the default openrobot package that has a bug on friction cones.

## Documentation

The documentation of **Crocoddyl** of its last release is available [here](https://gepettoweb.laas.fr/doc/loco-3d/crocoddyl/master/doxygen-html/).

## Citing Crocoddyl

To cite **Crocoddyl** in your academic research, please use the following bibtex lines:
```bibtex
@inproceedings{mastalli20crocoddyl,
  author={Mastalli, Carlos and Budhiraja, Rohan and Merkt, Wolfgang and Saurel, Guilhem and Hammoud, Bilal
  and Naveau, Maximilien and Carpentier, Justin and Righetti, Ludovic and Vijayakumar, Sethu and Mansard, Nicolas},
  title={{Crocoddyl: An Efficient and Versatile Framework for Multi-Contact Optimal Control}},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year={2020}
}
```
and the following one to reference this website:
```bibtex
@misc{crocoddylweb,
   author = {Carlos Mastalli, Rohan Budhiraja and Nicolas Mansard and others},
   title = {Crocoddyl: a fast and flexible optimal control library for robot control under contact sequence},
   howpublished = {https://github.com/loco-3d/crocoddyl/wikis/home},
   year = {2019}
}
```

**Crocoddyl** contributions go beyond efficient software implementation as well. Please also consider to cite the algorithm contributions of our different solvers and formulations:
 - Feasibility-driven DDP (FDDP): [[1]](#1)
 - Control-limited feasibility-driven DDP (Box-FDDP): [[2]](#2)
 - Multi-phase rigid optimal control: [[3]](#3)


Finally, please also consider citing **[Pinocchio](https://github.com/stack-of-tasks/pinocchio)**, which contributes to the efficient implementation of rigid body algorithms and their derivatives. For more details how to cite Pinocchio visit: [https://github.com/stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio).

Below, there is list of the selected publications that describe different components of **Crocoddyl**. For a complete list see [PUBLICATIONS.md](https://github.com/loco-3d/crocoddyl/blob/master/PUBLICATIONS.md).


### Selected publications
<a id="1">[1]</a>
C. Mastalli, R. Budhiraja, W. Merkt, G. Saurel, B. Hammoud, M. Naveau, J. Carpentier, L. Righetti, S. Vijayakumar and N. Mansard. [Crocoddyl: An Efficient and Versatile Framework for Multi-Contact Optimal Control](https://cmastalli.github.io/publications/crocoddyl20icra.html), IEEE International Conference on Robotics and Automation (ICRA), 2020

<a id="2">[2]</a>
C. Mastalli, W. Merkt, J. Marti-Saumell, H. Ferrolho, J. Sola, N. Mansard, S. Vijayakumar. [A Direct-Indirect Hybridization Approach to Control-Limited DDP](https://arxiv.org/pdf/2010.00411.pdf), 2021

<a id="3">[3]</a>
R. Budhiraja, J. Carpentier, C. Mastalli and N. Mansard. [Differential Dynamic Programming for Multi-Phase Rigid Contact Dynamics](https://cmastalli.github.io/publications/mddp18.html), IEEE RAS International Conference on Humanoid Robots (ICHR), 2018


## Questions and Issues

You have a question or an issue? You may either directly open a [new issue](https://github.com/loco-3d/crocoddyl/issues) or use the mailing list <crocoddyl@laas.fr>.


## Steering Committee

**Crocoddyl** is being managed by a steering committee which meets every two weeks to discuss the ongoing developments.

The committee is being led by [Carlos Mastalli](https://cmastalli.github.io/) (University of Edinburgh) and [Rohan Budhiraja](https://scholar.google.com/citations?user=NW9Io9AAAAAJ) (LAAS-CNRS).
[Nicolas Mansard](http://projects.laas.fr/gepetto/index.php/Members/NicolasMansard) (LAAS-CNRS), [Guilhem Saurel](http://projects.laas.fr/gepetto/index.php/Members/GuilhemSaurel) (LAAS-CNRS) and [Justin Carpentier](https://jcarpent.github.io/) (INRIA) are other members of the committee.


## Credits

The following people have been involved in the development of **Crocoddyl**:

- [Nicolas Mansard](http://projects.laas.fr/gepetto/index.php/Members/NicolasMansard) (LAAS-CNRS): project instructor and main developer
- [Carlos Mastalli](https://cmastalli.github.io/) (University of Edinburgh): main developer
- [Rohan Budhiraja](https://scholar.google.com/citations?user=NW9Io9AAAAAJ) (LAAS-CNRS): main developer
- [Justin Carpentier](https://jcarpent.github.io/) (INRIA): efficient analytical rigid-body dynamics derivatives, conda integration
- [Andrea Del Prete](https://andreadelprete.github.io/) (UNITN): feature extension
- [Maximilien Naveau](https://scholar.google.fr/citations?user=y_-cGlUAAAAJ&hl=fr) (MPI): unit-test support
- [Guilhem Saurel](http://projects.laas.fr/gepetto/index.php/Members/GuilhemSaurel) (LAAS-CNRS): continuous integration and deployment
- [Wolfgang Merkt](http://www.wolfgangmerkt.com/research/) (University of Oxford): feature extension and debugging
- [Josep Martí Saumell](https://www.iri.upc.edu/staff/jmarti) (IRI: CSIC-UPC): feature extension
- [Bilal Hammoud](https://scholar.google.com/citations?hl=en&user=h_4NKpsAAAAJ) (MPI): features extension
- [Julian Eßer](https://github.com/julesser) (DFKI): features extension (contact stability)


## Acknowledgments

The development of **Crocoddyl** is supported by the [EU MEMMO project](http://www.memmo-project.eu/), and the [EU RoboCom++ project](http://robocomplusplus.eu/).
It is maintained by the [Gepetto team](http://projects.laas.fr/gepetto/) [@LAAS-CNRS](http://www.laas.fr), the [Statistical Machine Learning and Motor Control Group](http://wcms.inf.ed.ac.uk/ipab/slmc) [@University of Edinburgh](https://www.edinburgh-robotics.org/), and the [Willow team](https://www.di.ens.fr/willow/) [@INRIA](https://www.inria.fr/fr/centre-inria-de-paris).
