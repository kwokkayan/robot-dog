##Set up requirment:
1. Google account (for creating Firebase projects)

##How to run
1. npm i to install dependencies
2. Configure .env:
   VITE_ROS_PACKAGE=public
   VITE_URDF_PATH=public/quadruped.urdf
   VITE_WS_URL=websocket url for middleware
   VITE_CAMERA_STREAM_URL=http url for video stream
   VITE_FIREBASE_API_KEY = API key of your firebase project
3. npm run dev to start web server

##How to get the API key 
1. Visit https://firebase.google.com/ and go to console
2. Create a new Firebase project
3. Go to Project Setting and create a web app
4. Firebase will generate API key, appId for you
5. Replace the codes in firebase.js with your web app's Firebase configuration
   (See [Add Firebase to your JavaScript project](https://firebase.google.com/docs/web/setup#add_firebase_to_your_app) )

