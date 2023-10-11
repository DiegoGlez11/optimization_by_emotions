const { app, BrowserWindow } = require('electron');

function createWindow() {
  // Create the browser window.
  let win = new BrowserWindow({
    width: 800,
    height: 1000,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
      devTools: true,
      webSecurity: true,
    }
  });

  win.maximize()

  // and load the index.html of the app.
  // win.loadFile('app/index.html');
  // win.loadURL("app/index.html");
  win.loadURL(`file://${__dirname}/app/emo_protocol.html`);
}

app.on('ready', createWindow);