

function create_table_obj(head) {

    let table = document.createElement("table");
    table.setAttribute("id", "inds_table");

    // head
    let th = document.createElement("tr");
    th.setAttribute("class", "t_head");
    for (let i = 0; i < head.length; i++) {
        let t = document.createElement("th");
        t.innerHTML = head[i];
        th.appendChild(t);
    }
    table.appendChild(th);
    table.onmouseover = mouseover_ind;
    table.onclick = click_ind;
    table.onmouseleave = mouseleave_ind;

    let container = document.createElement("div");
    container.setAttribute("id", "container_table_obj")
    container.appendChild(table);


    // if (EMOTIONS_CAPTURE_MODE == "traditional") {
    let txt = document.createElement("div");
    txt.setAttribute("id", "msg_table_obj");
    txt.innerHTML = "Click en la relación de tu preferencia";
    container.appendChild(txt);
    // }

    let btn_next = document.createElement("a");
    btn_next.setAttribute("class", "next round");
    btn_next.setAttribute("id", "next_individual");
    btn_next.innerHTML = "siguiente robot &#8250;";
    btn_next.onclick = function () {
        if (Object.keys(ds_fronts).length == 0) {
            alert("No hay individuos cargados");
        } else {
            if (SELECTION_MODE == "sel_automatic") return;

            if (!DATA_CONTROL.select_ind) {
                // console.log("event front from btn_next_individual");
                send_data_front();
            } else {
                console.error("Ejecutando individuo");
            }
        }
    }

    if (EMOTIONS_CAPTURE_MODE == "traditional") {
        btn_next.style.visibility = "hidden";
        txt.style.visibility = "";
    } else {
        btn_next.style.visibility = "";
        txt.style.visibility = "hidden";
    }

    container.appendChild(btn_next);


    // return table;
    return container;
}


function table_add_data(table, data_ind, num_inds, id_pareto) {
    let aux = table.childNodes[0];

    table.innerHTML = "";
    table.appendChild(aux);
    table.setAttribute("id_pareto", id_pareto);

    // data
    for (let i = 0; i < data_ind["x"].length; i++) {
        let tr = document.createElement("tr");
        tr.setAttribute("class", "row");
        tr.setAttribute("id", `${id_pareto}/ind_${num_inds[i]}`);
        tr.setAttribute("id_pareto", id_pareto);
        tr.setAttribute("num_ind", num_inds[i]);
        if (i % 2 != 0) tr.classList.add("pair_r");

        ["x", "y", "z"].forEach((key) => {
            let td = document.createElement("td");
            td.innerHTML = data_ind[key][i];
            tr.appendChild(td);

            if (key != "z") td.setAttribute("class", "data_td");
        });

        table.appendChild(tr);
    }
    return table;
}


function mouseover_ind(e) {
    let row = e.path[1];

    // si no es una fila 
    let c = row.getAttribute("class");
    if (c == undefined) return;
    if (c.search("row") < 0) return;

    // datos del ind
    let id_pareto = row.getAttribute("id_pareto");
    let num_ind = row.getAttribute("num_ind");
    // pos del ind
    let pos = pos_ind[id_pareto][`ind_${num_ind}`];
    // datos del frente
    let ds = ds_fronts[id_pareto];

    // si es ind seleccionado
    // let is_sel = ds.is_selected[pos];
    // si ya está seleccionado el ind
    // if (is_sel) {
    //     return;
    // }


    // cambio de color
    for (let i = 0; i < ds.line.color.length; i++) {
        ds.line.color[i] = 0.25;
    }
    ds.line.color[pos] = 1;

    // range
    let x = ds.x[pos];
    let min_all = Math.min(...ds.x);
    let max_all = Math.max(...ds.x);

    let min = constrain(x - 0.00000001, min_all, max_all);
    let max = constrain(x + 0.00000001, min_all, max_all);
    ds.dimensions[0].constraintrange = [min, max];


    // se actualiza la gráfica
    Plotly.react(GRAPH_OBJ_SPACE, Object.values(ds_fronts), get_layout(TYPE_GRAPH), { displaylogo: false, modeBarButtonsToRemove: ["zoom2d", 'lasso2d', "select2d", "toImage"] });
}

function mouseleave_ind(e) {
    let table = e.target;

    // si no es una fila 
    let c = table.getAttribute("id");
    if (c == undefined) return;
    if (c.search("inds_table") < 0) return;

    // datos del ind
    let id_pareto = table.getAttribute("id_pareto");
    // datos del frente
    let ds = ds_fronts[id_pareto];
    if (ds == undefined) return;


    // cambio de color
    let cont = 0;
    for (let i = 0; i < ds.line.color.length; i++) {
        if (!ds.is_selected[i])
            ds.line.color[i] = 0.25;
        else
            cont++;
    }
    if (cont < ds.line.color.length) {
        ds.dimensions[0].constraintrange = undefined;
        // se actualiza la gráfica
        Plotly.react(GRAPH_OBJ_SPACE, Object.values(ds_fronts), get_layout(TYPE_GRAPH), { displaylogo: false, modeBarButtonsToRemove: ["zoom2d", 'lasso2d', "select2d", "toImage"] });
    }
}

// solo funciona con modo tradicional y grafica parallel coordinates
function click_ind(e) {
    if (EMOTIONS_CAPTURE_MODE != "traditional") return;
    if (TYPE_GRAPH != "parcoords") return;
    if (DATA_CONTROL.select_ind) {
        console.error("Ejecutando individuo");
        return;
    }

    let row = e.path[1];

    // si no es una fila 
    let c = row.getAttribute("class");
    if (c == undefined) return;
    if (c.search("row") < 0) return;

    // id del frente
    let id_pareto = row.getAttribute("id_pareto");
    // datos del frente
    let ds = ds_fronts[id_pareto];
    // num del individuo
    let num_ind = row.getAttribute("num_ind");
    num_ind = parseInt(num_ind);
    // id del experimento
    let storage_pos = ds.storage_pos;
    // pos de los datos del ind
    let pos = pos_ind[id_pareto][`ind_${num_ind}`];

    // si es ind seleccionado
    let is_sel = ds.is_selected[pos];
    // si ya está seleccionado el ind
    if (is_sel) {
        console.log("Ya se seleccionó el ind", id_pareto, num_ind);
        return;
    }

    // cambio de estado de los puntos a seleccionados
    for (let i = 0; i < ds.is_selected.length; i++) {
        if (i != pos && !ds.is_selected[i]) {
            ds.is_selected[i] = true;
            // se actualizan los ind de la gráfica
            update_graph(id_pareto, ds.num_ind[i], false);
            // vista de la fila
            document.getElementById(`${id_pareto}/ind_${ds.num_ind[i]}`).style.background = "rgb(180, 179, 179)";
        }
    }

    // console.error("click in", id_pareto, "num_ind", num_ind, "storage_pos", storage_pos);
    // console.log("event front from click parcoords");

    // se eliminan los datos del frente
    DATA_FRONT = [];
    // se envía el ind seleccionado
    send_data_front(id_pareto, [num_ind], storage_pos);
}