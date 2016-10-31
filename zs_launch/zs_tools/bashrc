# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

# ROS zsROS_ws
# If you have versions of ROS, please comment next line and manually execute that command every time
source /opt/ros/indigo/setup.bash
export EDITOR='subl'
# source /home/zs/Downloads/TestingPackages/sim_ws/devel/setup.bash

# For multipule machine name resolution
# export ROS_IP=10.0.126.2
# export ROS_MASTER_URI=http://10.0.126.3:11311

# for new msg build
# (deprecated with rosbuild and be replaced with catkin) ???
#export ROS_PACKAGE_PATH=~/Documents/zsProjects/zsP3AT/zsROS_ws/:${ROS_PACKAGE_PATH}
#export ROS_WORKSPACE=~/Documents/zsProjects/zsP3AT/zsROS_ws/

# ROS 3dSimuation_ws
#export ROS_PACKAGE_PATH=/home/zs/Documents/zsProjects/zsP3AT/zssim_ws:${ROS_PACKAGE_PATH}
#export ROS_WORKSPACE=/home/zs/Documents/zsProjects/zsP3AT/zssim_ws/

# ROS Testing Packages
export ROS_WORKSPACE=/home/zs/Downloads/TestingPackages/sim_ws
export ROS_PACKAGE_PATH=${ROS_WORKSPACE}:${ROS_PACKAGE_PATH}

# to find the new executable, you have to update the config and source the setup.bash in your catkin_ws
# If you are using catkin, make sure you have sourced your workspace's setup.sh file after calling catkin_make but before trying to use your applications:

source ${ROS_WORKSPACE}/devel/setup.bash

# in new catkin build system it should be like this
# export CMAKE_PREFIX_PATH=~/Documents/zsProjects/zsP3AT/zsROS_ws/:${CMAKE_PREFIX_PATH}

# Naoqi
export PYTHONPATH=~/Documents/zsProjects/zsNao/zsROSNaoqi/pynaoqi-python2.7-2.1.4.13-linux64:$PYTHONPATH
export PATH=/home/zs/.local/bin:${PATH}

# OpenCL-Caffe
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/acml6.1.0/gfortran64/lib
export BOOST_ROOT=/usr/lib/x86_64-linux-gnu
export CLBALS_ROOT=/usr/local/lib64
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/acml6.1.0/gfortran64_mp/# lib

# Clion
export PATH=/home/zs/ProgramFile/clion-2016.1/bin:${PATH}

# Cyton Gamma 1500
# Using Viewer 4.0.20130130
#export EC_TOOLKITS="/usr/local/Robai/Cyton Gamma 1500 Viewer_4.0.20130130/toolkits"
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$EC_TOOLKITS/../bin
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/zs/Robai/Cyton Gamma 1500 Viewer_4.0.20130130/bin"
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/usr/local/Robai/Cyton Gamma 1500 Viewer_4.0.20130130"

# Using ros-cyton-package
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/
export EC_TOOLKITS="/home/zs/ProgramFile/Robai/Cyton Gamma 1500 Viewer_4.0.12-20160307/toolkits"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$EC_TOOLKITS/../bin

# Using CytonViewer 4.0.12
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$EC_TOOLKITS/../lib

# Gazebo!
#source /usr/share/drcsim/setup.sh
#export GAZEBO_MASTER_URI=http://localhost:11345
#export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models
#export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-2.2:/usr/share/gazebo_models
#export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-2.2/plugins
#export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/x86_64-linux-gnu/gazebo-2.2/plugins
#export OGRE_RESOURCE_PATH=/usr/lib/OGRE

# For usimg mobildrobots software
alias MobileSim="/usr/local/MobileSim/MobileSim"
