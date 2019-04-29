## Set up

Do the following commands. We recommend just compiling isam from irap svn's third-party

> $ mkdir isam_dev
> $ tar zxvfp isam_dev.tar.gz -C isam_dev #--strip 1
> $ cd isam_dev
> $ patch -p1 < ../isam_dev.patch
> $ make
> $ sudo make install
> $ sudo ldconfig
