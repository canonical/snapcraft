use http_body_util::{BodyExt, Full};
use hyper::body::Bytes;
use hyper::{Method, Request};
use hyper_util::client::legacy::Client;
use hyperlocal::{UnixClientExt, UnixConnector, Uri};
use std::error::Error;
use std::process::Command;
use std::collections::HashMap;
use std::path::Path;
use dotenv;

const SNAPD_SOCKET: &str = "/run/snapd-snap.socket";

async fn snapdapi_req() -> Result<serde_json::Value, Box<dyn Error + Send + Sync>> {
    let url: hyperlocal::Uri = Uri::new(SNAPD_SOCKET, "/v2/snapctl").into();

    let client: Client<UnixConnector, Full<Bytes>> = Client::unix();

    let snap_context = std::env::var("SNAP_CONTEXT")?;

    let request_body = format!(
        r#"{{"context-id":"{}","args":["get", "env", "envfile", "apps"]}}"#,
        snap_context
    );

    let req: Request<Full<Bytes>> = Request::builder()
        .method(Method::POST)
        .uri(url)
        .body(Full::from(request_body))?;

    let mut res = client.request(req).await?;

    let mut body: Vec<u8> = Vec::new();

    while let Some(frame_result) = res.frame().await {
        let frame = frame_result?;

        if let Some(segment) = frame.data_ref() {
            body.extend_from_slice(segment);
        }
    }

    Ok(serde_json::from_slice(&body)?)
}

fn process_env(env: &serde_json::Value) -> HashMap<String, String> {
    let obj = env.as_object()
        .ok_or("Expected an object (JSON input)").unwrap();
    let mut map = HashMap::new();

    for (k, v) in obj {
        if v.is_object() || v.is_array() {
            eprintln!(
                "Skipped invalid key containing dots: {}",
                v
            );
            continue;
        }

        let key = k.to_uppercase().replace("-", "_");
        let value = v.to_string();
        map.insert(key, value);
    }
    map
}

fn set_env_vars(app: &str, json: &serde_json::Value) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let stdout_str = json["result"]["stdout"].as_str().ok_or("Invalid stdout")?;
    let stdout_json: serde_json::Value = serde_json::from_str(stdout_str)?;


    if let Some(global_env) = stdout_json["env"].as_object() {
        for (key, value) in process_env(&serde_json::Value::Object(global_env.clone())) {
            std::env::set_var(key, value.trim_matches('"'));
        }
    }

    if let Some(app_env) = stdout_json["apps"][app]["env"].as_object() {
        for (key, value) in process_env(&serde_json::Value::Object(app_env.clone())) {
            std::env::set_var(key, value.trim_matches('"'));
        }
    }

    Ok(())
}

fn source_env_file(file_path: &str) -> std::io::Result<()> {
    let path = Path::new(file_path);

    if !path.exists() {
        eprintln!("File does not exist: {}", file_path);
        return Err(std::io::Error::new(std::io::ErrorKind::NotFound, "File does not exist"));
    }

    if !path.is_file() {
        eprintln!("File is not readable: {}", file_path);
        return Err(std::io::Error::new(std::io::ErrorKind::PermissionDenied, "File is not readable"));
    }

    dotenv::from_path(path).map_err(|e| {
        eprintln!("Failed to load environment file: {}", e);
        std::io::Error::new(std::io::ErrorKind::InvalidData, "Failed to load environment file")
    })?;

    Ok(())
}

fn set_env_vars_from_file(app: &str, json: &serde_json::Value) ->  Result<(), Box<dyn std::error::Error + Send + Sync>>  {
    // Extract the stdout JSON string and parse it
    let stdout_str = json["result"]["stdout"].as_str().ok_or("Invalid stdout")?;
    let stdout_json: serde_json::Value = serde_json::from_str(stdout_str)?;

    // Source the global envfile first
    if let Some(global_envfile) = stdout_json["envfile"].as_str() {
        source_env_file(global_envfile)?;
    }

    // Source the app-specific envfile
    if let Some(app_envfile) = stdout_json["apps"][app]["envfile"].as_str() {
        source_env_file(app_envfile)?;
    }

    Ok(())
}

#[tokio::main]
async fn run() -> Result<(), Box<dyn Error + Send + Sync>> {

    let json = snapdapi_req().await?;

    let app = std::env::var("env_alias")?;

    set_env_vars_from_file(&app, &json)?;
    set_env_vars(&app, &json)?;

    Ok(())
}

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {

    let args: Vec<String> = std::env::args().collect();

    if args.len() < 2 {
        eprintln!("Usage: {} <app-path>", args[0]);
        std::process::exit(1);
    }

    let command = args[1].clone();
    let args = args[2..].to_vec();

    run()?;

    let status = Command::new(command).args(args).status()?;

    std::process::exit(status.code().unwrap_or(1));
}
