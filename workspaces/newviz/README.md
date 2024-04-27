Set up requirment:
1. Google account

How to run
1. npm i to install dependencies
2. Configure .env:
    VITE_ROS_PACKAGE=public
    VITE_URDF_PATH=public/quadruped.urdf
    VITE_WS_URL=websocket url for middleware
    VITE_CAMERA_STREAM_URL=http url for video stream
    VITE_FIREBASE_API_KEY = API key of your firebase project
3. npm run dev to start web server

How to get the API key 
(https://firebase.google.com/docs/web/setup#add_firebase_to_your_app)
1. Visit https://firebase.google.com/ and go to console
2. Create a new Firebase project
3. Go to Project Setting and create a web app
4. Replace the Firebase SDK in firebase.js with the SDK of your project
