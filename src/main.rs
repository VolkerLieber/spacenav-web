use std::{collections::HashMap, ops::IndexMut, string};

use futures_util::{
    stream::{SplitSink, SplitStream},
    FutureExt, SinkExt, StreamExt,
};
use matrix::Matrix;
use spnav_posrot::Position;
use warp::{
    ws::{Message, WebSocket},
    Filter,
};

use rand::distributions::Alphanumeric;
use rand::{thread_rng, Rng};
use serde_json::{self, Number, Value};

use crate::matrix::MatrixOperationable;

mod matrix;
mod quat;
mod spnav_posrot;
mod vector;

#[tokio::main]
async fn main() {
    let websocket = warp::path::end()
        .and(warp::ws())
        .map(|ws: warp::ws::Ws| {
            // This will call our function if the handshake succeeds.
            ws.on_upgrade(move |socket| {
                // let (tx, rx) = socket.split();
                // // rx.forward(tx).map(|result| {
                // //     if let Err(e) = result {
                // //         eprintln!("websocket error: {:?}", e);
                // //     }
                // // })
                // send_welcome(&tx);

                // rx.forward(sink)
                handle_session(socket)
            })
        })
        .with(warp::reply::with::header("Sec-WebSocket-Protocol", "wamp"));

    // GET / -> index html
    let proxy = warp::path!("3dconnexion" / "nlproxy").map(|| "{\"port\":8181}");

    let routes = proxy.or(websocket).with(warp::reply::with::header(
        "Access-Control-Allow-Origin",
        "*",
    ));

    warp::serve(routes)
        .tls()
        .cert_path("C:\\Program Files (x86)\\3Dconnexion\\3DxWare\\3DxNLServer\\bin\\server.crt")
        .key_path("C:\\Program Files (x86)\\3Dconnexion\\3DxWare\\3DxNLServer\\bin\\server.key")
        .run(([127, 51, 68, 120], 8181))
        .await;
}

async fn send_welcome(tx: &mut SplitSink<WebSocket, Message>) {
    tx.send(Message::text(format!("[0,{:?},1,\"Nl-Proxy v1.4.3.19386 Copyright 2013-2022 3Dconnexion. All rights reserved.\"]", "8GXm6SS4smp3Ai0e"))).await.unwrap();
}

enum ClientReturnHandlers {
    SelectionEmpty,
    ViewPerspective,
    ViewAffine,
    ViewTarget,
}

struct spnav_event_motion {
    event_type: i32,
    x: i32,
    y: i32,
    z: i32,
    rx: i32,
    ry: i32,
    rz: i32,
    period: u32,
    // int *data;
}

struct Session {
    transmitter: SplitSink<WebSocket, Message>,
    receiver: SplitStream<WebSocket>,
    instance: u32,
    callbacks: HashMap<String, ClientReturnHandlers>,
    position: Position,
    view_matrix: Matrix,
    transactions: u32,
}
impl Session {
    fn new(socket: WebSocket) -> Session {
        let (session_tx, session_rx) = socket.split();
        Session {
            transmitter: session_tx,
            receiver: session_rx,
            instance: thread_rng().gen(),
            callbacks: HashMap::new(),
            position: Position::new(),
            view_matrix: [0.0; 16],
            transactions: 1,
        }
    }
}

async fn handle_session(socket: WebSocket) {
    println!("NEW SESSION");

    let mut session = Session::new(socket);

    send_welcome(&mut session.transmitter).await;

    while let Some(result) = session.receiver.next().await {
        let msg = match result {
            Ok(msg) => msg,
            Err(e) => {
                println!("WS ERROR: {e}");
                break;
            }
        };
        handle_msg(msg, &mut session).await;
    }
}

fn parse_msg<'a>(msg: Message) -> Result<(MessageType, Value), ()> {
    // Parse as json
    let json: Value = serde_json::from_slice(msg.as_bytes()).map_err(|err| ())?;

    // Expect array
    let json_array = json.as_array().ok_or(())?;

    // Expect length
    match json_array.len() {
        2..=6 => (),
        _ => return Err(()),
    };

    // Parse MessageType
    let msgType: MessageType = match &json_array[0] {
        Value::Number(msgType) => match msgType.as_u64() {
            Some(msgType) if msgType <= 8 => MessageType::from_u64(msgType),
            _ => return Err(()),
        },
        _ => return Err(()),
    };

    Ok((msgType, json))
}

async fn handle_msg(msg: Message, session: &mut Session) {
    let mut msg_text = msg.to_str().unwrap().to_string();
    msg_text.truncate(60);
    println!("MESSAGE: {:?}", msg_text);

    let (msgType, msg) = match parse_msg(msg) {
        Ok((msgType, json)) => (msgType, json),
        Err(_) => return,
    };

    let json = msg.as_array().expect("Unwrapping is handled in parse_msg");

    let ret = match msgType {
        MessageType::Welcome => return, // Server
        MessageType::Prefix => return,  // Client: Can be ignored
        MessageType::Call => handle_call(json, session).await,
        MessageType::CallResult => {
            println!("CallResult: {:?}", json);
            let callback_id = match json[1].as_str() {
                Some(callback_id) => callback_id,
                None => return,
            };
            let return_handler = match session.callbacks.remove(callback_id) {
                Some(return_handler) => return_handler,
                None => return,
            };
            match return_handler {
                ClientReturnHandlers::SelectionEmpty => todo!(),
                ClientReturnHandlers::ViewPerspective => todo!(),
                ClientReturnHandlers::ViewAffine => {
                    let view_affine = json[2].as_array().unwrap();
                    for (i, v) in session.view_matrix.iter_mut().enumerate() {
                        *v = view_affine[i].as_f64().unwrap() as f32;
                    }

                    let id = generate_id();
                    session
                        .callbacks
                        .insert(id.clone(), ClientReturnHandlers::ViewTarget);
                    session
                        .transmitter
                        .send(build_read_call(session.instance, &id, "view.target"))
                        .await;
                }
                ClientReturnHandlers::ViewTarget => {
                    let view_target = json[2].as_array().unwrap();
                    for (i, v) in session.position.pos.iter_mut().enumerate() {
                        *v = view_target[i].as_f64().unwrap() as f32;
                    }
                    session
                        .transmitter
                        .send(build_update_call(
                            session.instance,
                            &generate_id(),
                            "motion",
                            "true",
                        ))
                        .await;
                }
            };
            return;
        }
        MessageType::CallError => {
            println!("CallError: {:?}", json);
            return;
        }
        MessageType::Subscribe => {
            println!("Subscribe: {:?}", json);

            // Init variables
            // tx.send(build_update_call(
            //     instance,
            //     &generate_id(),
            //     "self:read",
            //     "selection.empty",
            // ))
            // .await;

            // tx.send(build_update_call(
            //     instance,
            //     &generate_id(),
            //     "self:read",
            //     "view.perspective",
            // ))
            // .await;

            // tx.send(build_update_call(
            //     instance,
            //     &generate_id(),
            //     "self:read",
            //     "view.affine",
            // ))
            // .await;

            // tx.send(build_update_call(
            //     instance,
            //     &generate_id(),
            //     "self:read",
            //     "view.target",
            // ))
            // .await;

            // [8,"3dconnexion:3dcontroller/6884113743086",[2,"t3bLavZGeDHtHSly","self:update","","motion",true]]

            let id = generate_id();
            session
                .callbacks
                .insert(id.clone(), ClientReturnHandlers::ViewAffine);

            session
                .transmitter
                .send(build_read_call(session.instance, &id, "view.affine"))
                .await;

            return;
        }
        MessageType::Unsubscribe => {
            println!("Unsubscribe: {:?}", json);
            return;
        }
        MessageType::Publish => {
            println!("Publish: {:?}", json);
            return;
        }
        MessageType::Event => return, // Server
    };

    let success = match ret {
        Ok(ret) => session.transmitter.send(ret).await,
        Err(_) => return,
    };

    match success {
        Ok(_) => return,
        Err(_) => println!("ERROR WHILE SENDING"),
    };
}

/*
[2,"0.wu6w9bnqe4","3dx_rpc:create","3dconnexion:3dmouse","0.6.0"]
[2,"0.h2jd78cvemc","3dx_rpc:update","3dconnexion:3dcontroller/6884113743086",{"commands":{"activeSet":"Part Studio"}}]
[2,"0.pim5f32a7ff","3dx_rpc:update","3dconnexion:3dcontroller/6884113743086",{"focus":true}]
 */
async fn handle_call(json: &Vec<Value>, session: &mut Session) -> Result<Message, ()> {
    // if (json.len() !== )

    let msg_id = json[1].as_str().ok_or(())?;

    let fn_name: &str = json[2].as_str().ok_or(())?;

    Ok(match fn_name {
        "3dx_rpc:create" => match json[3].as_str().ok_or(())? {
            "3dconnexion:3dmouse" => {
                build_result(msg_id, format!("{{\"connexion\":\"{}\"}}", generate_id()))
            }
            "3dconnexion:3dcontroller" => {
                build_result(msg_id, format!("{{\"instance\":{}}}", session.instance))
            }
            _ => {
                println!("UNHANDLED CALL: {:?}", json);
                return Err(());
            }
        },
        "3dx_rpc:update" => {
            println!("UPDATE:");
            if let Value::Object(map) = &json[4] {
                if let Some((_, Value::Object(frame))) = map.get_key_value("frame") {
                    if let Some((_, Value::Number(time))) = frame.get_key_value("time") {
                        session
                            .transmitter
                            .send(build_update_call(
                                session.instance,
                                &generate_id(),
                                "transaction",
                                &session.transactions.to_string(),
                            ))
                            .await;

                        session.transactions += 1;

                        session.position.move_obj(&spnav_event_motion {
                            event_type: 0,
                            x: 0,
                            y: 0,
                            z: 0,
                            rx: 50,
                            ry: 0,
                            rz: 0,
                            period: 0,
                        });

                        session.view_matrix.view(&session.position);

                        session
                            .transmitter
                            .send(build_update_call(
                                session.instance,
                                &generate_id(),
                                "view.affine",
                                &format!("{:?}", session.view_matrix),
                            ))
                            .await;

                        session
                            .transmitter
                            .send(build_update_call(
                                session.instance,
                                &generate_id(),
                                "transaction",
                                "0",
                            ))
                            .await;

                        session
                            .transmitter
                            .send(build_update_call(
                                session.instance,
                                &generate_id(),
                                "motion",
                                "false",
                            ))
                            .await;
                    }
                }
            }

            build_result(msg_id, "{}".to_string())
        }
        _ => {
            println!("UNHANDLED CALL: {:?}", json);
            return Err(());
        }
    })
}

/*
{
  "_version": "0.8.2.1",
  "browserNotSupportedMessage": "Browser does not support WebSockets (RFC6455)",
  "_idchars": "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789",
  "_idlen": 16,
  "_subprotocol": "wamp",
  "_debugrpc": false,
  "_debugpubsub": false,
  "_debugws": false,
  "_debugconnect": false,
  "_MESSAGE_TYPEID_WELCOME": 0,
  "_MESSAGE_TYPEID_PREFIX": 1,
  "_MESSAGE_TYPEID_CALL": 2,
  "_MESSAGE_TYPEID_CALL_RESULT": 3,
  "_MESSAGE_TYPEID_CALL_ERROR": 4,
  "_MESSAGE_TYPEID_SUBSCRIBE": 5,
  "_MESSAGE_TYPEID_UNSUBSCRIBE": 6,
  "_MESSAGE_TYPEID_PUBLISH": 7,
  "_MESSAGE_TYPEID_EVENT": 8,
  "CONNECTION_CLOSED": 0,
  "CONNECTION_LOST": 1,
  "CONNECTION_RETRIES_EXCEEDED": 2,
  "CONNECTION_UNREACHABLE": 3,
  "CONNECTION_UNSUPPORTED": 4,
  "CONNECTION_UNREACHABLE_SCHEDULED_RECONNECT": 5,
  "CONNECTION_LOST_SCHEDULED_RECONNECT": 6,
  "_UA_FIREFOX": {},
  "_UA_CHROME": {},
  "_UA_CHROMEFRAME": {},
  "_UA_WEBKIT": {},
  "_UA_WEBOS": {}
}
 */

#[derive(Debug)]
enum MessageType {
    /// 0: Server greets client
    Welcome,
    /// 1: Client greets server
    Prefix,
    /// 2: Client calls server function
    Call,
    /// 3: Return of server/client function
    CallResult,
    /// 4: Client sends event notice to server
    CallError,
    /// 5: Client joined room specified by server
    Subscribe,
    /// 6
    Unsubscribe,
    /// 7
    Publish,
    /// 8: Server calls client function
    Event,
}
impl MessageType {
    fn from_u64(value: u64) -> MessageType {
        match value {
            0 => MessageType::Welcome,
            1 => MessageType::Prefix,
            2 => MessageType::Call,
            3 => MessageType::CallResult,
            4 => MessageType::CallError,
            5 => MessageType::Subscribe,
            6 => MessageType::Unsubscribe,
            7 => MessageType::Publish,
            8 => MessageType::Event,
            _ => {
                println!("UNEXPECTED MESSAGE TYPE: {value}");
                MessageType::Welcome
            }
        }
    }
}

/*
8GXm6SS4smp3Ai0e
 */
fn generate_id() -> String {
    thread_rng()
        .sample_iter(&Alphanumeric)
        .take(16)
        .map(char::from)
        .collect()
}

/*
[0,"8GXm6SS4smp3Ai0e",1,"Nl-Proxy v1.4.3.19386 Copyright 2013-2022 3Dconnexion. All rights reserved."]
 */
fn build_welcome(id: &str) -> Message {
    Message::text(format!(
        "[{},{},1,\"Spacenav Proxy\"]",
        MessageType::Welcome as u32,
        id
    ))
}

/*
[3,"0.pim5f32a7ff",{}]
 */
fn build_result(id: &str, data: String) -> Message {
    Message::text(format!(
        "[{},\"{}\",{}]",
        MessageType::CallResult as u32,
        id,
        data
    ))
}

/*
[8,"3dconnexion:3dcontroller/6884113743086",[2,"1cJSqNoRqbVxr4Ds","self:update","","hit.selectionOnly",false]]
[8,"3dconnexion:3dcontroller/6884113743086",[2,"DMr5ZjiANwTtk65Y","self:update","","settings.changed",2]]
[8,"3dconnexion:3dcontroller/6884113743086",[2,"AEZWTPeAMmbGLPIB","self:read","","coordinateSystem"]]
 */
fn build_read_call(instance: u32, id: &str, key: &str) -> Message {
    Message::text(format!(
        "[{},\"3dconnexion:3dcontroller/{}\",[{},\"{}\",\"{}\",\"\",\"{}\"]]",
        MessageType::Event as u32,
        instance,
        MessageType::Call as u32,
        id,
        "self:read",
        key
    ))
}

fn build_update_call(instance: u32, id: &str, key: &str, value: &str) -> Message {
    Message::text(format!(
        "[{},\"3dconnexion:3dcontroller/{}\",[{},\"{}\",\"{}\",\"\",\"{}\",{}]]",
        MessageType::Event as u32,
        instance,
        MessageType::Call as u32,
        id,
        "self:update",
        key,
        value
    ))
}

/*
[8,"3dconnexion:3dcontroller/6884113743086",[2,"xqys5A4ZiD8E3lla","self:read","","selection.empty"]]
[8,"3dconnexion:3dcontroller/3042851224",[2,"HxMA6bihFAoFhhHK","self::read","","selection.empty"]]
 */
