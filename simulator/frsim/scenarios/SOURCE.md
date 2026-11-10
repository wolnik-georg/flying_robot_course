# Vendored scenario code

`curves.py`, `poly4d.py`, `scenarios.py`, `safety.py` are copied unchanged from the crazyswarm2 repository
(`crazyflie_examples/crazyflie_examples/formations/`, branch `study_FBL_stock`, last commit touching the folder: `e048019b1d7510b130178703a9de595c63ad562f`).
They import only numpy (no ROS). sha256 (first 16 characters): dbf8dda9f8164ff3 curves.py; a84f56ce05098235 poly4d.py; 05cf1a05db80f923 scenarios.py; 48ec2330f94d065f safety.py; 
`__init__.py`, `cells.py` and `motion.py` are new (package glue, the tuning / study cell lists, the evaluation of a scenario along time).

The vendored files keep their original formatting (some lines are longer than 120 characters); only the package code outside this folder is wrapped at 120.
