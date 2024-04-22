


function create_table_obj(head) {

    // creación de la tabla
    let table = document.createElement("table");
    table.setAttribute("id", "inds_table");

    // fila de encabezado
    let th = document.createElement("tr");
    th.setAttribute("class", "t_head");
    for (let i = 0; i < head.length; i++) {
        let t = document.createElement("th");
        t.innerHTML = head[i];
        th.appendChild(t);
    }
    //  se agrega el encabezado
    table.appendChild(th);

    // eventos de la tabla
    table.onmouseover = mouseover_ind;
    table.onclick = click_ind;
    table.onmouseleave = mouseleave_ind;

    // contenedor
    let container = document.createElement("div");
    container.setAttribute("id", "container_table_obj");

    // texto debajo de la tabla
    let txt = document.createElement("div");
    txt.setAttribute("id", "msg_table_obj");
    txt.innerHTML = "Click en la relación de tu preferencia";

    // se agreagn los elementos
    container.appendChild(txt);
    container.appendChild(table);

    // btn siguiente
    let btn_next = document.createElement("a");
    btn_next.setAttribute("class", "next round");
    btn_next.setAttribute("id", "next_individual");
    btn_next.innerHTML = "siguiente robot &#8250;";
    btn_next.onclick = function () {
        if (Object.keys(PARETO_DATA).length == 0) {
            alert("No hay individuos cargados");
        } else {
            if (SELECTION_MODE == "sel_automatic") return;

            if (!CONTROL_DATA.select_ind) {
                // console.log("event front from btn_next_individual");
                send_data_front();
            } else {
                console.error("Ejecutando individuo");
            }
        }
    }

    // se oculta o muestra el btn siguiente
    if (EMOTIONS_CAPTURE_MODE == "traditional") {
        btn_next.style.visibility = "hidden";
        txt.style.visibility = "";
    } else {
        btn_next.style.visibility = "";
        txt.style.visibility = "hidden";
    }
    container.appendChild(btn_next);

    return container;
}


function table_set_data(table, ds, id_pareto) {
    let aux = table.childNodes[0];

    table.innerHTML = "";
    table.appendChild(aux);

    // data
    for (let i = 0; i < ds["x"].length; i++) {
        let tr = document.createElement("tr");
        tr.setAttribute("class", "row");
        tr.setAttribute("position", i);
        tr.setAttribute("relative_id_pareto", ds.relative_id_pareto[i]);
        tr.setAttribute("id_ind", `${ds.relative_id_pareto[i]}/ind_${ds.relative_num_ind[i]}`);
        tr.setAttribute("id", `${ds.relative_id_pareto[i]}/ind_${ds.relative_num_ind[i]}`);

        if (i % 2 != 0) tr.classList.add("pair_r");

        ["x", "y", "obj3"].forEach((key) => {
            let td = document.createElement("td");
            td.innerHTML = ds[key][i];
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
    let id_pareto = row.getAttribute("relative_id_pareto");
    // pos del ind
    let pos = row.getAttribute("position");

    // datos del frente
    let ds = PARETO_DATA[id_pareto];

    // cambio de color
    for (let i = 0; i < ds.line.color.length; i++) {
        if (ds.is_selected[i]) continue;
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
    Plotly.react(GRAPH_OBJ_SPACE, Object.values(PARETO_DATA), get_layout(TYPE_GRAPH), { displaylogo: false, modeBarButtonsToRemove: ["zoom2d", 'lasso2d', "select2d", "toImage"] });
}

function mouseleave_ind(e) {
    let table = e.target;

    // si no es una fila 
    let c = table.getAttribute("id");
    if (c == undefined) return;
    if (c.search("inds_table") < 0) return;

    // datos del ind
    let id_pareto = table.getAttribute("relative_id_pareto");

    // datos del frente
    let ds = PARETO_DATA[id_pareto];
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
        Plotly.react(GRAPH_OBJ_SPACE, Object.values(PARETO_DATA), get_layout(TYPE_GRAPH), { displaylogo: false, modeBarButtonsToRemove: ["zoom2d", 'lasso2d', "select2d", "toImage"] });
    }
}


// solo funciona con modo tradicional y gráfica parallel coordinates
function click_ind(e) {
    if (EMOTIONS_CAPTURE_MODE != "traditional") return;
    if (TYPE_GRAPH != "parcoords") return;

    if (CONTROL_DATA.select_ind) {
        console.error("Ejecutando individuo");
        return;
    }

    let row = e.path[1];

    // si no es una fila 
    let c = row.getAttribute("class");
    if (c == undefined) return;
    if (c.search("row") < 0) return;

    // id del frente
    let id_pareto = row.getAttribute("relative_id_pareto");
    // pos del ind
    let pos = row.getAttribute("position");

    // datos del frente
    let ds = PARETO_DATA[id_pareto];

    // si es ind seleccionado
    let is_sel = ds.is_selected[pos];
    // si ya está seleccionado el ind
    if (is_sel) {
        console.log("Ya se seleccionó el ind", id_pareto, num_ind);
        return;
    }

    // cambio de estado de los puntos seleccionados
    for (let i = 0; i < ds.is_selected.length; i++) {
        if (i != pos) {
            ds.is_selected[i] = true;
            // se actualizan los ind de la gráfica
            update_graph(id_pareto, ds.relative_id_pareto[i], ds.relative_num_ind[i], false);

            // vista de la fila
            let elem = document.getElementById(`${ds.relative_id_pareto[i]}/ind_${ds.relative_num_ind[i]}`);
            // console.log(elem);
            elem.style.background = "rgb(180, 179, 179)";
        }
    }
    // vista de la fila
    document.getElementById(`${id_pareto}/ind_${ds.relative_num_ind[pos]}`).style.background = "blue";


    // se envía el ind seleccionado
    send_data_front(id_pareto, ds.relative_id_pareto[pos], ds.relative_num_ind[pos], ds.storage_pos);
}