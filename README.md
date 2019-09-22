# Package tum_ardrone_iitb

This package contains the implementation corresponding to the following publications:

- [Mosaicing of Multiplanar Regions throughAutonomous Navigation of Off-the-shelfQuadcopter](Meghshyam G. Prasad, Sonapraneeth Akula, Anirudh Vemula, Sharat Chandran)

This Package builds on the well known paper Camera-Based Navigation of a Low-Cost Quadrocopter by Engel et al. presented in International Conference on Intelligent Robots and Systems, 2012. Please study the http://wiki.ros.org/tum_ardrone and the corresponding paper for more information on this part of the software. Also, be aware of the license that comes with it. 

## Results
<p align="center">
!["Aircrafts"](http://meghshyam.github.io/multiplanar/figures/aircrafts2.png)
</p>
**Fig 1:** *(a) Large illustrations of military aircrafts were exhibited on a wall. (b) Images captured by our autonomous navigation algorithm are mosaiced in a single panorama as shown in (c).*

**Aircraft** We have performed an experiment where two posters were put on a long planar wall. The arrangement is shown in Fig. 1(a). 

In the path planning stage, overall 8 positions are estimated to encompass the whole area. Images captured from those positions (shown
in Fig. 1(b) are mosaiced using our algorithm to get the final output as shown in the Fig. 1(c).

Notice that the gap in the wall between the two illustrations are genuinely maintained in Fig. 1(c) for accurate portrayal. 

**Mixed**: We have performed an experiment where the posters were arranged in mixed fashion. The arrangement is shown in
Fig. 2 (Top-Left) where the middle posters form the concave region while the side posters form the convex region. 

The selected area for imaging is shown in Fig. 2 (Top-Right). In path planning overall 24 (6 from each plane) positions are estimated to encompass the user-selected area. Images captured from those positions are mosaiced using our algorithm to get the final output as shown in the Fig. 2 (Bottom).

!["dominating_sets_example2"](http://meghshyam.github.io/multiplanar/figures/mixed2Result.png)

**Fig 2:** *Top-left: An exhibition of Chinese lanterns' posters arranged in mixed (convex and concave) fashion is shown in the long-range photograph. We wish to image all posters orthographically with details. Top-right: The green quadrilateral shows the user selected area, while the blue, violet magenta, and red colored quadrilaterals represent the corresponding multiple bounded planar regions estimated by our algorithm. Bottom: Images captured by our autonomous navigation algorithm are mosaiced.*


## License

The major part of this software package - that is everything except PTAM - is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html. PTAM (comprised of all files in /src/stateestimation/PTAM) has it's own licence, see http://www.robots.ox.ac.uk/~gk/PTAM/download.html. This licence in particular prohibits commercial use of the software.
