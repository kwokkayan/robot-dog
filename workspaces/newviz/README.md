## Prerequisite
1. Google account (for creating Firebase projects)


## Configuration
Configure [environement file] (.env):
- VITE_ROS_PACKAGE=public
- VITE_URDF_PATH=public/quadruped.urdf
- VITE_WS_URL= Websocket URL for ROS connection
- VITE_CAMERA_STREAM_URL= URL for live camera stream
- VITE_FIREBASE_API_KEY = API key of your firebase project


## Launch the Web
1. run shell command to install dependencies
```npm i```
2. run shell command to start web server
```npm run dev```


## How to get the API key 
1. Visit [Firebase](https://firebase.google.com/) and go to console
2. Create a new Firebase project
3. Go to Project Setting and create a web app
4. Firebase will generate API key, appId for you
5. Replace the codes in firebase.js with your web app's Firebase configuration
   (See [Add Firebase to your JavaScript project](https://firebase.google.com/docs/web/setup#add_firebase_to_your_app) )

