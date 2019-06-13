curdir=`pwd`
cd wiringPi/
git reset --hard 96344ff7125182989f98d3be8d111952a8f74e15
cd wiringPi/ && make static
cd $curdir
make
exit 0
