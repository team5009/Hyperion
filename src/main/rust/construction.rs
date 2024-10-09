use std::{f64::consts::PI, vec};

use crate::{Command, CommandState, Event, Point, Spline, Wait, WaitType};
use serde_json::{json, Value};

static ERR: &str = "Invalid Path: ";

pub fn command_read(content: String) -> Result<Value, ()> {
    let mut commands: Vec<Command> = vec![];
    // let mut error: Option<Error> = Option::None;

    let content = content.to_lowercase();
    let content = content.replace(' ', "");
    let content = content.lines();
    let mut execution_state = CommandState::Start;
    let mut spline_state = CommandState::None;
    let mut block_state = CommandState::None;


    for (i, line) in content.clone().enumerate() {
        let mut line = line.split(',').collect::<Vec<&str>>();
        let name = line.first().unwrap().to_string();
        let event_call = line.last().unwrap().to_string();
        line.remove(0);
        line.pop();

        let args = line;

        if (i == 0 && name != "start") || (i == content.clone().count() && name != "end") {
            panic!("{}Path must start with start and end with end", ERR);
        }

        match name.as_str() {
            "start" => {
                if execution_state != CommandState::Start {
                    panic!("{}No start command", ERR)
                }

                let args = args
                    .iter()
                    .map(|x| {
                        x.parse::<f64>()
                            .expect("Invalid Path: Cannot parse START arguments into numbers")
                    })
                    .collect::<Vec<f64>>();

                let event = Event::new(event_call);

                let mut point = Point::new_from_vec(args, event);
                point.rot *= PI / 180.0;

                commands.push(Command::Start(point));

                execution_state = CommandState::Reading;
            }
            "end" => {
                if execution_state != CommandState::Reading {
                    panic!("{}No start command", ERR);
                }

                let event = Event::new(event_call);

                commands.push(Command::End(event));

                execution_state = CommandState::End;
            }
            "wait" => {
                if execution_state != CommandState::Reading {
                    panic!("{}No start command", ERR);
                }

                if spline_state == CommandState::Start {
                    match commands.last_mut().unwrap() {
                        Command::Spline(spline) => match spline.last_mut() {
                            Some(_) => {
                                let waiting =
                                    check_time_or_event(args.first().unwrap().to_string());
                                let wait = Wait::new(waiting, Event::new(event_call));

                                spline.push(Spline::Wait(wait));
                            }
                            None => {
                                panic!("{}Unable to add Wait to Spline command list", ERR);
                            }
                        },
                        _ => {
                            panic!("{}Unable to add wait to Spline command list", ERR);
                        }
                    }
                } else {
                    let waiting = check_time_or_event(args.first().unwrap().to_string());
                    let wait = Wait::new(waiting, Event::new(event_call));

                    commands.push(Command::Wait(wait));
                }
            }
            "line" => {
                if execution_state != CommandState::Reading {
                    panic!("{}No start command", ERR);
                }

                let args = args
                    .iter()
                    .map(|x| {
                        x.parse::<f64>()
                            .expect("Invalid Path: Cannot parse LINE arguments into numbers")
                    })
                    .collect::<Vec<f64>>();

                let event = Event::new(event_call);

                let mut point = Point::new_from_vec(args, event);
                point.rot *= PI / 180.0;

                match block_state {
                    CommandState::Reading => match commands.last_mut().unwrap() {
                        Command::Block(block) => {
                            block.push(Command::Line(point));
                        }
                        _ => {
                            panic!("{}Unable to add line to command list", ERR);
                        }
                    },
                    _ => {
                        commands.push(Command::Line(point));
                    }
                }
            }
            "spline" => {
                if execution_state != CommandState::Reading {
                    panic!("{}No start command", ERR);
                }

                match spline_state {
                    CommandState::None => {
                        commands.push(Command::Spline(vec![]));
                        spline_state = CommandState::Start;
                    }

                    CommandState::Start => {
                        spline_state = CommandState::None;
                    }

                    _ => {
                        panic!("{}Unable to add spline to command list", ERR);
                    }
                }
            }
            "bezier" => {
                if execution_state != CommandState::Reading {
                    panic!("{}No start command", ERR);
                }

                let args = args
                    .iter()
                    .map(|x| {
                        x.parse::<f64>()
                            .expect("Invalid Path: Cannot parse BEZIER arguments into numbers")
                    })
                    .collect::<Vec<f64>>();

                let event = Event::new(event_call);

                let mut point = Point::new_from_vec(args, event);
                point.rot *= PI / 180.0;

                match spline_state {
                    CommandState::Start => match commands.last_mut().unwrap() {
                        Command::Spline(spline) => {
                            let mut bezier = crate::Bezier::new();
                            bezier.start(point);
                            spline.push(Spline::Bezier(bezier));

                            spline_state = CommandState::Reading;
                        }
                        _ => {
                            panic!("{}Unable to add bezier to command list", ERR);
                        }
                    },
                    CommandState::Reading => match commands.last_mut().unwrap() {
                        Command::Spline(spline) => match spline.last_mut() {
                            Some(spline) => match spline {
                                Spline::Bezier(bezier) => {
                                    bezier.end(point);
                                    spline_state = CommandState::Start;
                                }
                                _ => {
                                    panic!("{}", ERR);
                                }
                            },
                            None => {
                                panic!("{}", ERR);
                            }
                        },
                        _ => {
                            panic!("{}Unable to end bezier from command list", ERR);
                        }
                    },
                    _ => {
                        panic!("{}Spline Error", ERR);
                    }
                }
            }

            "control" => {
                if execution_state != CommandState::Reading {
                    panic!("{}No start command", ERR);
                }

                let args = args
                    .iter()
                    .map(|x| {
                        x.parse::<f64>()
                            .expect("Invalid Path: Cannot parse BEZIER arguments into numbers")
                    })
                    .collect::<Vec<f64>>();

                let event = Event::new(event_call);

                let mut point = Point::new_from_vec(args, event);
                point.rot *= PI / 180.0;

                match spline_state {
                    CommandState::Start => {
                        panic!("{}Can't add control without a Spline and Bezier", ERR);
                    }
                    CommandState::Reading => {
                        match commands.last_mut().expect("Invalid command list") {
                            Command::Spline(spline) => match spline.last_mut() {
                                Some(spline) => match spline {
                                    Spline::Bezier(bezier) => {
                                        bezier.add_control(point);
                                        spline_state = CommandState::Reading;
                                    }
                                    _ => {
                                        panic!("{}", ERR)
                                    }
                                },
                                None => {
                                    panic!("{}", ERR);
                                }
                            },
                            _ => {
                                panic!("{}Last command is not a Spline", ERR);
                            }
                        }
                    }
                    _ => {
                        panic!("{}Unable to add control", ERR);
                    }
                }
            }
            "block" => {
                if execution_state != CommandState::Reading {
                    panic!("{}No start command", ERR);
                }
                if spline_state != CommandState::None {
                    panic!("{}Can't add block to Spline", ERR);
                }

                match block_state {
                    CommandState::None => {
                        commands.push(Command::Block(vec![]));
                        block_state = CommandState::Reading;
                    }
                    CommandState::Reading => {
                        block_state = CommandState::None;
                    }
                    _ => {
                        panic!("{}Unable to add block to command list", ERR);
                    }
                }
            }
            _ => {
                panic!("{}{} is not a valid command", ERR, name);
            }
        }
    }
    Ok(json!(commands))
}

pub fn check_time_or_event(text: String) -> WaitType {
    let mut split_text = text.split("").collect::<Vec<&str>>();
    split_text.remove(0);
    let number_set = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"];
    let wait_type = if number_set.contains(split_text.first().unwrap()) {
        LocalWaitType::Time
    } else {
        LocalWaitType::Event
    };

    match wait_type {
        LocalWaitType::Time => {
            let mut number = String::new();
            let mut time = String::new();

            for i in split_text {
                if number_set.contains(&i) {
                    number.push_str(i);
                } else {
                    time.push_str(i);
                }
            }

            // println!("{} {}", number, time);
            let number = number.parse::<u64>().expect("Invalid Path: Invalid time");
            let time = time.to_string();

            match time.as_str() {
                "ms" => WaitType::Time(number),
                "sec" => WaitType::Time(number * 1000),
                "min" => WaitType::Time(number * 1000 * 60),
                "hour" => WaitType::Time(number * 1000 * 60 * 60),
                "day" => WaitType::Time(number * 1000 * 60 * 60 * 24),
                "week" => WaitType::Time(number * 1000 * 60 * 60 * 24 * 7),
                "month" => WaitType::Time(number * 1000 * 60 * 60 * 24 * 7 * 4),
                "year" => WaitType::Time(number * 1000 * 60 * 60 * 24 * 7 * 4 * 12),
                _ => {
                    panic!("Invalid time");
                }
            }
        }
        LocalWaitType::Event => WaitType::Event(Event::new(text)),
    }
}

enum LocalWaitType {
    Time,
    Event,
}
