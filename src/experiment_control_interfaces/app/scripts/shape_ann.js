//-----------------------------------------------------------------
//------------------arquitectura del neurocontrolador -------------
//-----------------------------------------------------------------


function create_hidde(val = undefined) {
    let c = document.createElement("div");
    c.setAttribute("class", "hidde_ann_layer");

    let in_h = document.createElement("input");
    in_h.setAttribute("type", "number");
    in_h.setAttribute("class", "hidde_ann_in");
    if (val == undefined) in_h.setAttribute("value", 0);
    else in_h.setAttribute("value", val);

    let btn = document.createElement("input");
    btn.setAttribute("type", "button");
    btn.setAttribute("class", "hidde_ann_btn");
    btn.setAttribute("value", "Eliminar");
    btn.addEventListener("click", (e) => {
        c.remove();
    });

    c.appendChild(in_h);
    c.appendChild(btn);
    return c;
}

function get_shape_ann_hmtl() {
    let n_in = parseFloat(document.getElementById("in_ann").value);
    let n_out = parseFloat(document.getElementById("out_ann").value);
    let n_h = document.getElementsByClassName("hidde_ann_in");
    let shape = [n_in];
    for (let i = 0; i < n_h.length; i++) {
        let v = parseFloat(n_h[i].value);
        if (v > 0 && !isNaN(v)) shape.push(v);
        else {
            alert("EL número de neuronas de las capas del neurocontrolador deben ser mayor a cero");
            return;
        }
    }
    shape.push(n_out);
    return shape;
}



async function get_shape_ann() {
    let n_in = document.getElementById("in_ann");
    let n_out = document.getElementById("out_ann");
    let n_h = document.getElementById("hidde_ann_container");
    n_h.innerHTML = "";

    let srv = rosnodejs.nh.serviceClient("/control_optimization", "nsga2_dmaking/control_optimization");
    let msg = Object.assign({}, srv_control_optimization);
    msg.action = "get_shape_ann";
    srv.call(msg).then((res) => {
        n_in.value = res.shape_ann[0];
        n_out.value = res.shape_ann.pop();
        for (let i = 1; i < res.shape_ann.length; i++) {
            let e = create_hidde(res.shape_ann[i]);
            n_h.appendChild(e);
        }
    });
}



let add_ = document.getElementById("add_hidde");
add_.addEventListener("click", (e) => {
    let c = document.getElementById("hidde_ann_container");
    let elem = create_hidde();
    c.appendChild(elem);
})

create_input_ros("button", "Actualizar neurocontrolador", "/control_optimization", "nsga2_dmaking/control_optimization",
    (btn) => {
        let msg = Object.assign({}, msg_control_optimization);
        msg.action = "set_shape_ann";
        msg.shape_ann = get_shape_ann_hmtl();

        if (msg.shape_ann != undefined) return msg;
        else return;
    },
    (res) => {
        get_shape_ann().then(() => {
            show_blink(document.getElementById("ann_shape"), color_success);
        }).catch(() => {
            show_blink(document.getElementById("ann_shape"), color_error);
        });

    }, "Actualizar la arquitectura del neurocontrolador").then((btn) => {
        document.getElementById("control_shape").appendChild(btn);
    });


get_shape_ann();