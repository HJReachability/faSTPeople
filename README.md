# faSTPeople
FaSTrack + STP + people, oh my!

## Dependencies
This repository depends upon the `fastrack` repository, which may be found [here](https://github.com/HJReachability/fastrack), as well as the `crazyflie_human` package which may be found [here](https://github.com/abajcsy/crazyflie_human). Since `crazyflie_human` is a standalone package and not part of a workspace, you will need to add it to this workspace as follows (from the top level directory):
```
cd ros/src
git clone https://github.com/abajcsy/crazyflie_human
```

To obtain `fastrack`, please follow the directions in it's README. Remember to add it to your `ROS_PACKAGE_PATH` by running
```
source <path-to-fastrack>/ros/devel/setup.bash
```

## Usage
This repository is structured as a ROS workspace, which is located in the `ros/` directory. As with any othe ROS workspace, it may be built by running the following command from the `ros/` directory:
```
catkin_make
```

## Development guidelines

### Style
Please follow the following style guidelines while developing this repository.
* Use the Google C++ style guide for C++11. This may be found [here](https://google.github.io/styleguide/cppguide.html).
* There's a great tool called `clang-format` which you can use to auto-format your C++ to follow different style guides. It can fix things like spacing and line wrapping, but it can also do much more. Please use it if you're not sure about style conventions. Even if you are sure, please use it anyway. [Here](https://electronjs.org/docs/development/clang-format)'s a guide on how to set it up.

### Pull requests
This repository is set up so that code must be _reviewed_ by someone with write or admin privileges before it can make it into the `master` branch. This serves two purposes: first, it forces us to know what each other are working on at a very granular level, and second, it helps enforce style and code quality. To get your code reviewed, you will need to create a new branch for the feature you're working on, commit your code there, open a pull request (PR) to `master`, request review(s) from one or more team members with write privileges, and upon approval merge into `master`. A typical workflow might look like this:
1. You want to start working on a new feature, so you open up a new branch: `git checkout -b <your-git-id>/<feature-name>`
2. Make some commits as you work on this branch, e.g. `git commit -am '<commit-msg>' && git push origin <your-git-id>/<feature-name>`
3. When you're ready to merge back into `master`, first do a `rebase`. This will help keep track of any bugs that get introduced into the `master` branch. To do this, checkout the latest `master`, pull, checkout your branch, and rebase:
    ```
    git checkout master
    git pull origin master
    git checkout <your-git-id>/<feature-name>
    git rebase origin/master
    git push --force origin <your-git-id>/<feature-name>
    ```
4. Now you're ready to open a PR. Go to the [web interface](https://github.com/HJReachability/faSTPeople) for this repository, find your branch, and open a PR. Write up a short description of your feature, add any release notes you want, and add reviewers. Once they approve your PR you can merge.
