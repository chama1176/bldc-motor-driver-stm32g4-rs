pub trait PeriodicTask {
    fn run(&self);
}

// app側にinterruptを実装し，g4側から呼び出す．
