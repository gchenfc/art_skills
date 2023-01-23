# SOP

1. start server.py using
    ```sh
    cd whiteboard
    python server.py
    ```
    Take note of the ip address listed on "Serving whiteboard at: xxx.xxx.xxx.xxx:5900.  Ignore the 5900 part, but copy the ip address.
   1. In [client/main.js](client/main.js), paste the ip address into "HOST"
2. (optional) start fit_node.py by running `python fit_node.py` from the `art_skills/python/src` directory.
3. (optional) connect robot:  
    Either:
     * From the [cable-robot repo (html branch)](https://github.gatech.edu/borglab/cable-robot/tree/feature/html_control_panel%2Fmain), start a live server (see 4) and open the site/index.html file, or
     * open the html file directly in a browser, e.g. URL:
        <a href="file:///Users/gerry/GIT_REPOS/cable-robot/site/index.html">file:///Users/gerry/GIT_REPOS/cable-robot/site/index.html</a>
     * calibrate & run `ts0.3` to set the speed to 0.3.
4. start whiteboard "Live Server":
   1. Install the [Live Server](https://marketplace.visualstudio.com/items?itemName=ritwickdey.LiveServer) vscode extension, or have some other way of serving a webpage over a local network.
   2. Start the Live Server (bottom-right of the screen is a "Go Live" icon), take note of the port number.
   3. In the browser (or iPad) navigate to the whiteboard/client/index.html page.  Depending what the root of your vscode workspace was when you started the live-server, the exact path might be slightly different, but hopefully it's pretty intuitive.  The url is:  
          `[ip address]:[live server port]/whiteboard/client`  
      So, for example,  
          `143.215.82.42:5501/whiteboard/client/`  
      Note: although "localhost" or "127.0.0.1" will work on the same computer that started live-server, it will *not* work on any other device, so you need to use the IP address copied in step 1.
