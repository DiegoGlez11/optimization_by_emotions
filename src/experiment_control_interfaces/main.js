// const { app, BrowserWindow } = require("electron");
const { app, ipcMain, BrowserWindow, session, dialog } = require('electron')

function createWindow() {

  // Create the browser window.
  let win = new BrowserWindow({
    width: 1700,
    height: 1000,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
      devTools: true,
      enableRemoteModule: true,

      backgroundThrottling: false,
      nativeWindowOpen: false,
      webSecurity: false,
    },
  });


  // win.setFullScreen(true);

  // and load the index.html of the app.
  win.loadURL(`file://${__dirname}/app/html/control_experiment.html`);

  win.webContents.openDevTools();
}

app.on("ready", createWindow);
