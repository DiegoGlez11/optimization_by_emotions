let graph_3d = document.getElementById("graph_3d");

let layout = {
    uirevision: 'true',
    xaxis: { autorange: true },
    yaxis: { autorange: true },
    margin: {
        l: 0,
        r: 0,
        b: 0,
        t: 30,
    }
};

let config = {
    toImageButtonOptions: {
        format: 'svg', // one of png, svg, jpeg, webp
        filename: 'custom_image',
        height: 700,
        width: 900,
        scale: 1 // Multiply title/legend/axis/canvas sizes by this factor
    }
};


Plotly.newPlot(graph_3d, [get_base_dataset("")], layout);

graph_3d.on('plotly_click', function (data) {
    if (data.points.length > 0) {
        let ind = data.points[0];

        let id_pareto = ind.data.name;
        let num_ind = ind.data.num_ind[ind.pointNumber];
        console.log(id_pareto, num_ind, ind);

        simulate_individual(id_pareto, num_ind);

        // let param = new user_interaction.experiment_control.Request();
        // param.control = "select_ind";
        // param.id_pareto_front = id_pareto;
        // param.num_inds = [num_ind];
        // experiment_control(param);



    }
});


function load_preference() {
    return new Promise((resolve, reject) => {
        let user_name = get_actual_user();
        // se cargan los ind seleccionados
        let path = `${root_dir}/${user_name}/history/selected_individuals.txt`;
        load_object(path).then(async (ind_selected) => {
            if (ind_selected.length == undefined) ind_selected = [];
            console.log(path);
            // registro de frentes seleccionados
            for (let n_sel = 0; n_sel < ind_selected.length; n_sel++) {
                const ind = ind_selected[n_sel];

                let param = new user_interaction.next_front_selected.Request();
                param.id_pareto_front = ind["id_pareto_front"];
                param.num_individuals = ind["num_individuals"];

                if (ind["reference_point"] != undefined) {
                    param.reference_point = ind["reference_point"]["point"];
                    param.num_ind_ref_point = ind["reference_point"]["num_ind"];
                    param.id_pareto_front_ref = ind["reference_point"]["id_pareto_front"];
                }

                if (ind["close_individual"] != undefined) {
                    param.close_individual = ind["close_individual"]["point"];
                    param.num_ind_close = ind["close_individual"]["num_ind"];
                    param.id_pareto_front_close = ind["close_individual"]["id_pareto_front"];
                }

                // await plot_front(param).catch((e) => {
                //     console.error("Error al mostrar el frente seleccionado" + id_front, e);
                //     reject();
                // });
            }
        });
    });
}


// ------------------------------------------------------
// RECIBE LOS INDIVIDUOS DEL SIGUIENTE FRENTE 
// let ds_fronts_loaded = { "path": get_base_dataset("path", "lines+markers") };
let ds_fronts_loaded = {};
let ds_fronts_pref = {};
rosnodejs.nh.advertiseService("/next_front_selected", "user_interaction/next_front_selected", async function (msg) {
    let id_pareto = msg.id_pareto_front;
    let num_inds = msg.num_individuals;

    let ref_point = msg.reference_point;
    let num_ref = msg.num_ind_ref_point;
    let id_pareto_ref = msg.id_pareto_front_ref;

    let close_point = msg.close_individual;
    let num_close = msg.num_ind_close;
    let id_pareto_close = msg.id_pareto_front_close;


    // ------------------------------------------------------
    // valores visuales de los puntos en la gráfica 3D:
    // ["circle", "circle-open", "cross", "diamond", "diamond-open", "square", "square-open", "x"]

    // si no esta cargado
    let ds;
    if (ds_fronts_loaded[id_pareto] == undefined) {
        // se obtiene el espacio de los objetivos del frente
        await get_obj_space(id_pareto).then((res) => {
            // modo de la gráfica
            let mode;
            if (is_lines.checked) mode = "lines+markers";
            else mode = "markers";

            // dataset
            ds = get_base_dataset(id_pareto, mode);
            ds.marker["color"] = randomColor({ format: "rgba", alpha: 0.3, luminosity: "dark" });
            ds["num_ind"] = [];

            // 1d to 3d
            for (let i = 0; i < res.pop_size; i++) {
                let d = res.obj_space.splice(0, res.num_obj);
                ds.x.push(d[0]);
                ds.y.push(d[1]);
                ds.z.push(d[2]);
                ds.text.push(`ind_${i}<br>${id_pareto}`);
                ds.marker.symbol.push("circle");
                ds.num_ind.push(i);
            }
            // se agrega el dataset
            ds_fronts_loaded[id_pareto] = ds;
        }).catch((e) => {
            console.error("Error al cargar el frente en 3D\n\n", e);
            return;
        });
    } else {
        // existe el dataset
        ds = ds_fronts_loaded[id_pareto];
    }

    let id = `sel_${id_pareto}`;
    let ds_sel = get_base_dataset(id, "lines+markers");
    ds_sel.marker.line.width = 40;
    ds_sel.marker["color"] = [];
    ds_sel.marker.line["color"] = [];
    ds_sel["num_ind"] = [];
    // ds_ds_selind["showlegend"] = false;
    ds_sel.marker.size = [];


    // punto de referencia
    if (ref_point.length > 0) {
        ds_fronts_loaded[id] = ds_sel;
        // let ds_ref = ds;
        let ds_ref = ds_fronts_loaded[id];
        // se agrega el punto
        ds_ref.x.push(ref_point[0]);
        ds_ref.y.push(ref_point[1]);
        ds_ref.z.push(ref_point[2]);
        // visual
        ds_ref.text.push(`REF_ind_${num_ref}<br>${id_pareto_ref}`);
        ds_ref.marker.symbol.push("x");
        ds_ref.marker.line.color.push("red");
        ds_ref.marker.color.push("red");
        ds_ref.marker.size.push(5);
        ds_ref.num_ind.push(num_ref);
    }

    // individuo más cercano en el siguiente frente
    if (close_point.length > 0) {
        ds_fronts_loaded[id] = ds_sel;
        let ds_close = ds_fronts_loaded[id];

        ds_close.x.push(close_point[0]);
        ds_close.y.push(close_point[1]);
        ds_close.z.push(close_point[2]);
        ds_close.text.push(`CLOSE_ind_${num_close}<br>${id_pareto_close}`);
        ds_close.marker.symbol.push("circle-open");
        ds_close.marker.line.color.push("blue");
        ds_close.marker.color.push("blue");
        ds_close.marker.size.push(6);
        ds_close.num_ind.push(num_close);
    }

    // si hay individuos seleccionados
    if (num_inds.length > 0) {
        ds_fronts_loaded[id] = ds_sel;
        ds_fronts_pref[id] = true;
        ds_fronts_pref[id_pareto] = true;

        // let ds_ind = ds_fronts_loaded["path"];
        let ds_ind = ds_fronts_loaded[id];
        for (let n_ind = 0; n_ind < num_inds.length; n_ind++) {
            // individuo a cambiar sus popiedades
            let n_i = num_inds[n_ind];

            // ds.text[n_i] = `SEL_ind_${n_i}<br>${id_pareto}`;
            // ds.marker.symbol[n_i] = "diamond";
            // ds.marker.line.color[n_i] = "red";
            ds.marker.size[n_i] = 0;

            ds_ind.x.push(ds.x[n_i]);
            ds_ind.y.push(ds.y[n_i]);
            ds_ind.z.push(ds.z[n_i]);
            ds_ind.text.push(`SEL_ind_${n_i}<br>${id_pareto}`);
            ds_ind.marker.symbol.push("diamond");
            ds_ind.marker.line.color.push("green");
            // ds_ind.marker.color.push("rgba(39,128,23,0.8)");
            ds_ind.marker.color.push(ds.marker.color[n_i]);
            ds_ind.marker.size.push(3);
            ds_ind.num_ind.push(n_i);

            if (ds.marker["color"] != undefined) {
                let color = ds.marker.color.split(",");
                // cambio alpha           
                color[3] = "1)";
                ds.marker.color = color.join();
            }
        }
    }


    Plotly.react(graph_3d, Object.values(ds_fronts_loaded), layout, config);

    return true;
});


// -----------------------------------------------
// ----------------------------------------------
// ----------------------------------------------
// ----- FUNCIONES PARA EL RANGO DOBLE -----------
// -----------------------------------------------
// ----------------------------------------------
// código en: https://medium.com/@predragdavidovic10/native-dual-range-slider-html-css-javascript-91e778134816



function controlFromInput(fromSlider, fromInput, toInput, controlSlider) {
    const [from, to] = getParsed(fromInput, toInput);
    fillSlider(fromInput, toInput, '#C6C6C6', '#25daa5', controlSlider);
    if (from > to) {
        fromSlider.value = to;
        fromInput.value = to;
    } else {
        fromSlider.value = from;
    }
}

function controlToInput(toSlider, fromInput, toInput, controlSlider) {
    const [from, to] = getParsed(fromInput, toInput);
    fillSlider(fromInput, toInput, '#C6C6C6', '#25daa5', controlSlider);
    setToggleAccessible(toInput);
    if (from <= to) {
        toSlider.value = to;
        toInput.value = to;
    } else {
        toInput.value = from;
    }
}

function controlFromSlider(fromSlider, toSlider, fromInput) {
    const [from, to] = getParsed(fromSlider, toSlider);
    fillSlider(fromSlider, toSlider, '#C6C6C6', '#25daa5', toSlider);
    if (from > to) {
        fromSlider.value = to;
        fromInput.value = to;
    } else {
        fromInput.value = from;
    }
}

function controlToSlider(fromSlider, toSlider, toInput) {
    const [from, to] = getParsed(fromSlider, toSlider);
    fillSlider(fromSlider, toSlider, '#C6C6C6', '#25daa5', toSlider);
    setToggleAccessible(toSlider);
    if (from <= to) {
        toSlider.value = to;
        toInput.value = to;
    } else {
        toInput.value = from;
        toSlider.value = from;
    }
}

function getParsed(currentFrom, currentTo) {
    const from = parseInt(currentFrom.value, 10);
    const to = parseInt(currentTo.value, 10);
    return [from, to];
}

function fillSlider(from, to, sliderColor, rangeColor, controlSlider) {
    const rangeDistance = to.max - to.min;
    const fromPosition = from.value - to.min;
    const toPosition = to.value - to.min;
    controlSlider.style.background = `linear-gradient(
    to right,
    ${sliderColor} 0%,
    ${sliderColor} ${(fromPosition) / (rangeDistance) * 100}%,
    ${rangeColor} ${((fromPosition) / (rangeDistance)) * 100}%,
    ${rangeColor} ${(toPosition) / (rangeDistance) * 100}%, 
    ${sliderColor} ${(toPosition) / (rangeDistance) * 100}%, 
    ${sliderColor} 100%)`;
}

function setToggleAccessible(currentTarget) {
    const toSlider = document.querySelector('#toSlider');
    if (Number(currentTarget.value) <= 0) {
        toSlider.style.zIndex = 2;
    } else {
        toSlider.style.zIndex = 0;
    }
}




function create_double_slider(container, ini_range_slider = 1, end_range_slider = 60) {

    let DOUBLE_RANGE = `
  <div class="range_container">
      <div class="sliders_control">
          <input id="fromSlider" type="range" value=${end_range_slider} min=${ini_range_slider} max=${end_range_slider} />
          <input id="toSlider" type="range" value=${end_range_slider} min=${ini_range_slider} max=${end_range_slider} />
      </div>
      <div class="form_control">
          <div class="form_control_container">
              <div class="form_control_container__time">Min</div>
              <input class="form_control_container__time__input" type="number" id="fromInput" value=${end_range_slider} min=${ini_range_slider} max=${end_range_slider} />
          </div>
          <div class="form_control_container">
              <div class="form_control_container__time">Max</div>
              <input class="form_control_container__time__input" type="number" id="toInput" value=${end_range_slider} min=${ini_range_slider} max=${end_range_slider} />
          </div>
      </div>
  </div>`;

    // SE AGREGA EL CÓDIGO HTML EN SU CONTENEDOR
    container.innerHTML = DOUBLE_RANGE;

    const fromSlider = document.querySelector('#fromSlider');
    const toSlider = document.querySelector('#toSlider');
    const fromInput = document.querySelector('#fromInput');
    const toInput = document.querySelector('#toInput');
    fillSlider(fromSlider, toSlider, '#C6C6C6', '#25daa5', toSlider);
    setToggleAccessible(toSlider);

    fromSlider.oninput = () => controlFromSlider(fromSlider, toSlider, fromInput);
    toSlider.oninput = () => controlToSlider(fromSlider, toSlider, toInput);
    fromInput.oninput = () => controlFromInput(fromSlider, fromInput, toInput, toSlider);
    toInput.oninput = () => controlToInput(toSlider, fromInput, toInput, toSlider);

    return { from: fromSlider, to: toSlider };
}

// -----------------------------------------
// -----------------------------------------
// FUNCIONALIDADES AGREGADAS AL DOUBLE RANGE
// -----------------------------------------
// -----------------------------------------

let is_lines = document.getElementById("is_lines");
// cambio en el modo de vista
is_lines.addEventListener("change", () => {
    let mode;
    if (is_lines.checked) mode = "lines+markers";
    else mode = "markers";

    let array_ds = Object.values(ds_fronts_loaded);
    for (let i = 0; i < array_ds.length; i++) {
        let ds = array_ds[i];
        if (ds.name.search("sel") < 0) ds.mode = mode;
    }
    Plotly.react(graph_3d, array_ds, layout, config);
});


let is_mesh = document.getElementById("is_mesh");
// cambio en el modo de vista
is_mesh.addEventListener("change", () => {
    let type;
    if (is_mesh.checked) type = "mesh3d";
    else type = "scatter3d";

    let array_ds = Object.values(ds_fronts_loaded);
    for (let i = 0; i < array_ds.length; i++) {
        let ds = array_ds[i];
        if (ds.name.search("sel") < 0) ds.type = type;
    }
    Plotly.react(graph_3d, array_ds, layout, config);
});



let btn_show = document.getElementById("btn_show");
btn_show.onclick = () => {
    let mode_visible;
    if (btn_show.innerHTML == "Mostrar") {
        mode_visible = true;
        btn_show.innerHTML = "Ocultar";
    } else {
        mode_visible = "legendonly";
        btn_show.innerHTML = "Mostrar";
    }

    let str_find = "";
    let is_sel = document.getElementById("is_sel");
    if (is_sel.checked) str_find = "sel";

    let array_ds = Object.values(ds_fronts_loaded);
    for (let i = 0; i < array_ds.length; i++) {
        let ds = array_ds[i];
        if (ds.name.search(str_find) >= 0)
            ds.visible = mode_visible;
    }
    Plotly.react(graph_3d, array_ds, layout, config);
}


document.getElementById("btn_update_pred").onclick = () => {
    ds_fronts_loaded = {};
    // reseteo de la gráfica
    Plotly.react(graph_3d, [get_base_dataset("")], layout, config)
    load_preference();
}



// ------------------------------------------------------
// ------------------------------------------------------
// ------ TABLA DE LOS FRENTES DE PARETO A USAR ---------
// ------------------------------------------------------
function create_fronts_table() {
    let table = [];
    for (let n_gen = 1; n_gen <= 60; n_gen++) {
        let id_pareto = `optimized-${n_gen}`;
        table.push(id_pareto);
    }
    save_object(table, `fronts_table.txt`, root_dir);

}
// create_fronts_table();


// ------------------------------------------------------
// ------------------------------------------------------


// se crea el slider doble
load_object(`${root_dir}/fronts_table.txt`).then((table) => {
    let controls = create_double_slider(document.getElementById("range_fronts"), 0, table.length);

    async function slider_load() {
        console.log(ds_fronts_pref);

        for (id_front in ds_fronts_loaded) {
            if (ds_fronts_pref[id_front] == undefined) delete ds_fronts_loaded[id_front];
        }
        console.log(ds_fronts_loaded);

        let from_value = parseInt(controls.from.value);
        let to_value = parseInt(controls.to.value);

        // se extraen los frentes
        let subFronts = table.slice(from_value - 1, to_value);
        // console.log(subFronts);
        // se cargan los frentes en la gráfica
        for (let n_front = 0; n_front < subFronts.length; n_front++) {
            let id_front = subFronts[n_front];

            let msg = new user_interaction.next_front_selected.Request();
            msg.id_pareto_front = id_front;
            await plot_front(msg).then(() => {
                // console.log("success", id_front);
            }).catch((e) => {
                console.error("Error al mostrar el frente " + id_front, e);
                return;
            });
        }

    }

    controls.from.onchange = slider_load;
    controls.to.onchange = slider_load;
});


// trata del evento  usando se crea el usuario
let in_user = document.getElementById("actual_user");
// evento cuando se crea el usuario
in_user.addEventListener("create_users", () => {
    // se cargan las nuevas preferencias
    load_preference();
}, false);



// evento cuando se cambia se usuario
in_user.addEventListener("change", () => {
    ds_fronts_loaded = {};
    Plotly.newPlot(graph_3d, [get_base_dataset("")], layout);
    load_preference();
});
