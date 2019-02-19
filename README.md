# LWB-Baseline

The Low-Power Wireless Bus (LWB) is a communication protocol for low-power wireless multi-hop networks that was published in the 10th ACM Conference on Embedded Networked Sensor Systems (SenSys 2012). It provides a shared-bus abstraction for higher layer protocols and hides the complexity of the underlying network.
Internally, LWB uses fast and reliable Glossy floods to exchange information within the network, thus supporting arbitrary traffic patterns.

The original code for the TelosB platform was never officially released. This repository provides a LWB reference implementation whose functionality and performance are as equivalent as possible to the original LWB.

# Branch: EWSN2019_competition

This branch features a slightly modified version of LWB in order to compete at the EWSN Dependability Competition 2019 in Beijing.

# Publications
Federico Ferrari, Marco Zimmerling, Luca Mottola, and Lothar Thiele. [Low-Power Wireless Bus](ftp://ftp.tik.ee.ethz.ch/pub/people/ferrarif/FZMT2012.pdf). In Proceedings of the 10th ACM Conference on Embedded Network Sensor Systems (SenSys), Toronto (Canada), November 2012.

Federico Ferrari, Marco Zimmerling, Lothar Thiele, and Olga Saukh. [Efficient Network Flooding and Time Synchronization with Glossy](ftp://ftp.tik.ee.ethz.ch/pub/people/ferrarif/FZTS2011.pdf). In Proceedings of the 10th ACM/IEEE International Conference on Information Processing in Sensor Networks (IPSN), Chicago (IL, USA), April 2011. Best Paper Award.

Fabian Mager, Romain Jacob, Reto Da Forno, and Marco Zimmerling. Competition: Low-Power Wireless Bus Baseline. ACM International Conference on Embedded Wireless Systems and Networks (EWSN), Beijing (China), February 2019.
