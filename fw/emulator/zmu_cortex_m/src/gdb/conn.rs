use gdbstub::conn::{Connection, ConnectionExt};
use std::{
    io::Read,
    net::{Shutdown, TcpListener, TcpStream},
    str,
};

pub struct TcpConnection {
    stream: TcpStream,
}

impl TcpConnection {
    pub fn new_localhost(port: u16) -> Result<TcpConnection, &'static str> {
        let listener = TcpListener::bind(("127.0.0.1", port)).unwrap();

        let (stream, _addr) = listener
            .accept()
            .map_err(|_| "Error accepting socket connection")?;

        stream
            .set_read_timeout(Some(std::time::Duration::from_millis(1)))
            .map_err(|_| "Error setting timeout")?;
        Ok(TcpConnection { stream })
    }
}

impl Drop for TcpConnection {
    fn drop(&mut self) {
        self.stream
            .shutdown(Shutdown::Both)
            .expect("shutdown failed");
    }
}

impl Connection for TcpConnection {
    type Error = &'static str;

    fn write(&mut self, b: u8) -> Result<(), &'static str> {
        match self.stream.write(b) {
            Ok(()) => Ok(()),
            Err(_) => Err("socket write failed"),
        }
    }

    fn flush(&mut self) -> Result<(), &'static str> {
        match self.stream.flush() {
            Ok(()) => Ok(()),
            Err(_) => Err("socket flush failed"),
        }
    }
}

impl ConnectionExt for TcpConnection {
    fn read(&mut self) -> std::result::Result<u8, Self::Error> {
        let mut buf: [u8; 1] = [0];
        loop {
            match self.stream.read_exact(&mut buf) {
                Ok(()) => break,
                Err(e) => match e.kind() {
                    #[cfg(windows)]
                    std::io::ErrorKind::TimedOut => {}
                    #[cfg(unix)]
                    std::io::ErrorKind::WouldBlock => {}
                    _ => return Err("socket read failed"),
                },
            }
        }
        Ok(buf[0])
    }

    fn peek(&mut self) -> std::result::Result<Option<u8>, Self::Error> {
        let mut buf: [u8; 1] = [0];
        loop {
            match self.stream.peek(&mut buf) {
                Ok(_) => break,
                Err(e) => match e.kind() {
                    #[cfg(windows)]
                    std::io::ErrorKind::TimedOut => {}
                    #[cfg(unix)]
                    std::io::ErrorKind::WouldBlock => {}
                    _ => return Err("socket peek failed"),
                },
            }
        }
        Ok(Some(buf[0]))
    }
}
