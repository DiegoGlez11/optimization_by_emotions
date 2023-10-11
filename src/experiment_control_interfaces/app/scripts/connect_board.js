//boton de actualizar
let elem = document.createElement("input");
elem.setAttribute("type", "button");
elem.setAttribute("value", "Actualizar");
// elem.setAttribute("class", "blink_show");

let btn_update = add_tooltip_and_blink(elem, "Actualizar los puertos");
// let btn_update = add_tooltip(elem, "Actualizar los puertos");
// let blink_ = document.createElement("div");
// blink_.setAttribute("class", "blink");
// blink_.appendChild(btn_update);
// let c_blink = document.createElement("div");
// c_blink.appendChild(blink_);
elem.addEventListener("click", () => {
    update_ports();
});


document.getElementById("container_update").appendChild(btn_update);

//se agregan los puertos 
function update_ports() {
    serialPort.list().then(function (ports) {
        //se agrega la opción de la simulación
        let op1 = document.createElement("option");
        op1.setAttribute("value", "simulacion");
        op1.innerHTML = "Simulación";

        document.getElementById("port_list").innerHTML = "";
        document.getElementById("port_list").appendChild(op1);

        ports.forEach(function (port) {
            // {
            //     "path": "/dev/ttyUSB0",
            //     "manufacturer": "FTDI",
            //     "serialNumber": "DM00D6XO",
            //     "pnpId": "usb-FTDI_FT231X_USB_UART_DM00D6XO-if00-port0",
            //     "vendorId": "0403",
            //     "productId": "6015"
            // }
            if (port.pnpId != undefined && port.serialNumber != undefined) {

                let op = document.createElement("option");
                op.setAttribute("value", port.path);
                op.innerHTML = port.path;
                document.getElementById("port_list").appendChild(op);
            }
        })
        show_blink(btn_update, color_success);
    }).catch(() => {
        show_blink(btn_update, color_error);
    });
}
update_ports();


create_input_ros("button", "conectar", "/connect_board", "eeg_signal_acquisition/connect_board",
    (btn) => {
        //obtenemos el puerto
        let port = document.getElementById("port_list");
        let name_port = port.options[port.selectedIndex].value;

        //mensaje ros
        let connect = { port_path: name_port };
        return connect;
    },
    (res) => {
        if (res.msg.search("Error") > -1) {
            alert(res.msg);
        } else {
            document.getElementById("connected").className = "init";
            // update_window_size();
        }
    }, "conectar con la tarjeta cyton").then((btn) => {
        document.getElementById("container_update").appendChild(btn);
    });



//se verifica el estado de conexión de la tarjeta 
addEventListener("readystatechange", function () {
    if (document.readyState == "complete") {
        get_state_acquisition().then((res) => {
            if (res.connected) {
                document.getElementById("connected").className = "init";
            } else {
                document.getElementById("connected").className = "stop";
            }
        });
    }
});




//---- boton desconectar  ---
let desc = document.createElement("input");
desc.setAttribute("type", "button");
desc.setAttribute("value", "Desconectar");
let btn_dis = add_tooltip_and_blink(desc, "Desconectar la tarjeta")

desc.addEventListener("click", () => {
    let srv_dis = rosnodejs.nh.serviceClient("/disconnect_board", "eeg_signal_acquisition/disconnect_board");

    srv_dis.call({}).then(() => {
        document.getElementById("connected").className = "stop";
    }).catch((e) => {
        console.error("error al detener la tarjeta", e);
    });
});
document.getElementById("container_update").appendChild(btn_dis);

