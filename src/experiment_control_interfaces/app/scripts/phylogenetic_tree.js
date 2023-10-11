


function create_node(id, tag) {
    //contenedores
    let container_childs = document.createElement("div");
    container_childs.setAttribute("class", "container_childs");
    container_childs.setAttribute("id", id);

    let container_parent = document.createElement("div");
    container_parent.setAttribute("class", "container_parent");

    let container_global = document.createElement("div");
    container_global.setAttribute("class", "container_global " + id);

    //nodo
    let node = document.createElement("div");
    node.setAttribute("class", "node");
    node.setAttribute("value", id)
    node.innerHTML = tag;

    //line
    let line = document.createElement("div");
    line.setAttribute("class", "line");

    //estructura
    container_parent.appendChild(node);
    container_parent.appendChild(line);
    container_global.appendChild(container_parent);
    container_global.appendChild(container_childs);

    return container_global;
}



function load_phylogenetic_tree() {
    let dir_root = [];
    new Promise((resolve, reject) => {

        //obtenemos todos los frentes de pareto
        let dir_front = homedir + "/catkin_ws/src/neurocontroller_database/database/" + get_actual_user() + "/pareto_fronts";
        fs.readdir(dir_front, { withFileTypes: true }, (error, files) => {
            if (error) {
                console.error(error);
                return;
            }
            const directoriesInDIrectory = files.filter((item) => {
                //se buscan todas las poblaciones root
                if (item.name.search("root") > -1) {
                    return item.isDirectory();
                }
            }).map((item) => item.name);
            dir_root = directoriesInDIrectory;


            //la ruta decodifica la ruta desde la raiz hasta su posición en el arbol filogenetico
            for (let i = 0; i < dir_root.length; i++) {
                //id del nodo actual
                let id_node = dir_root[i];

                //segmentación del id
                let front = id_node.split("_");
                //se elimina la palabra 'front' del id ya que no forma parte de él
                front.shift();
                let id_increment = "";
                let parent_id = "";
                while (front.length > 0) {
                    tag = front.shift();
                    id_increment += "_" + tag;
                    let node_actual = document.getElementById(id_increment);
                    if (node_actual == undefined) {
                        //nodo raiz
                        if (id_increment.split("_").length == 2) {
                            //contenedor
                            let container = document.getElementById("container_phylo_tree");
                            container.appendChild(create_node(id_increment, tag));
                        } else {
                            let node_parent = document.getElementById(parent_id);
                            node_parent.appendChild(create_node(id_increment, tag));
                        }
                    }
                    parent_id = id_increment;
                }
            }

            //Se elimina el margen de los hijos nodo que tengan mas de un hijo
            let childs = document.getElementsByClassName("container_childs");
            for (let j = 0; j < childs.length; j++) {
                if (childs[j].children.length < 2) {
                    for (let k = 0; k < childs[j].children.length; k++) childs[j].children[k].style.margin = "0";
                }
            }

            //eventos de los nodos del arbol
            let node_phylo = document.getElementsByClassName("node");

            for (let g = 0; g < node_phylo.length; g++) {
                node_phylo[g].addEventListener("click", () => {
                    let id_pareto = node_phylo[g].getAttribute("value");

                    //Se verifica que no éste cargado el frente de pareto
                    // for (let k = 0; k < ids_pareto_fronts.length; k++) {
                    //     if (ids_pareto_fronts[k] == id_pareto) {
                    //         show_blink(document.getElementById("canva"), color_success);
                    //         return;
                    //     }
                    // }

                    //se carga la población
                    // load_population_into_nsga2(id_pareto).then((res_load) => {
                    //     load_phylogenetic_data(id_pareto).then((phylo_l) => {
                    //         phylo_l["pos_database"] = globalChartRef.data.datasets.length;
                    //         update_fromArray1D(res_load.obj_space, phylo_l).then(() => {
                    //             select_pop(phylo_l);
                    //         });
                    //     });
                    // }).catch(() => {
                    //     show_blink(document.getElementById("canva"), color_error);
                    //     reject();
                    //     return;
                    // });
                    load_pareto_front(id_pareto).then(() => {
                        show_blink(document.getElementById("canva"), color_success);
                        return;
                    }).catch(() => {
                        show_blink(document.getElementById("canva"), color_error);
                        return;
                    });
                });
            }
            resolve();
        });
    });
}



