const { app, BrowserWindow } = require("electron");
const { ipcMain } = require('electron');

let width_w = 910;
let height_w = 1020;

function createWindow() {

  // Create the browser window.
  let win = new BrowserWindow({
    resizable: true,
    autoHideMenuBar: true,
    width: width_w,
    height: height_w,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
      devTools: true,
      webSecurity: true,
    },
  });

  win.webContents.setWindowOpenHandler((test) => {
    console.log(test);
    return {
      // trafficLightPosition: { x: 10, y: 10 },
      action: 'allow',
      overrideBrowserWindowOptions: {
        autoHideMenuBar: true,
        frame: true,
        // width: 900,
        // height: 1189,
        // fullscreenable: false,
        // webPreferences: {
        //   preload: 'effect_window.js'
        // }
      }
    };
  });


  //win.setFullScreen(true);

  // and load the index.html of the app.
  win.loadURL(`file://${__dirname}/app/html/graph_preference.html`);

  ipcMain.on('resize-me-please', (event, arg) => {
    let width = arg.width;
    let height = arg.height;
    let type_win = arg.type_window;

    // if (type_win != undefined) {
    //   width = width;
    // }



    if (width == undefined || height == undefined) {
      console.error(`Valores inválidos de ventana: (${width},${height})`);
      return;
    }

    win.setSize(width, height);
    win.setPosition(0, 0);
  })

}


app.on("ready", createWindow);


