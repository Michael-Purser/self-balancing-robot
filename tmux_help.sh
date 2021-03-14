#!/bin/bash

echo """
TMUX basic commands: see https://github.com/gpakosz/.tmux
- commands require a <prefix>, then a key: Ctrl+a = <prefix>
- a 'window' is like a tab
- a 'pane' is a subdivision of current terminal, like tiling
You can use keybindings below or click with mouse to select active pane/window, resize etc

Windows (ie tabs)
  <prefix> C-h and <prefix> C-l let you navigate windows
  <prefix> Tab brings you to the last active window

Panes (ie sub-parts of a tab)
  <prefix> - splits the current pane vertically
  <prefix> _ splits the current pane horizontally
  <prefix> h, <prefix> j, <prefix> k and <prefix> l let you navigate panes ala Vim
  <prefix> H, <prefix> J, <prefix> K, <prefix> L let you resize panes
  <prefix> < and <prefix> > let you swap panes
  <prefix> + maximizes the current pane to a new window

Other
  if you shutdown the terminal, the session keeps running (like screen). 
  reattach with 'tmux list-sessions' and 'tmux attach-session -t <session_name>'
"""