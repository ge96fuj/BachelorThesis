const {expect} = require('chai');
const sinon = require('sinon');

const {initConfig,addNewTrafficLight,handleReceivedMessage} = require('/Users/skanderjneyeh/Desktop/Bachelor_Thesis_Rep_6Mai/Traffic-Light-Server-Client/Server/server.js');
const TrafficGroup = require('/Users/skanderjneyeh/Desktop/Bachelor_Thesis_Rep_6Mai/Traffic-Light-Server-Client/Server/core/TrafficGroup');
const { TrafficLightStatus } = require('/Users/skanderjneyeh/Desktop/Bachelor_Thesis_Rep_6Mai/Traffic-Light-Server-Client/Server/core/TrafficLight');
const fs = require('fs');
const path = require('path');
const ConfigPath = '/Users/skanderjneyeh/Documents/Bachelor_thesis_rep/BachelorThesis/Server/config';
const ConfigFile = 'lightConfig.js';
const TestConfigPath = '/Users/skanderjneyeh/Documents/Bachelor_thesis_rep/BachelorThesis/Server/test/Configs'
global.lights = {};
global.trafficGroupsList = [];
const DEFAULT_DURATIONS = {
    red: 2000,
    yellow: 2000,
    green: 10000
  };




describe('Test1' , () => {
it('dummy test', () => {

});
});

describe('Configs_Test' , () => {
    it('testing config 1', () => {
        const testConfig = require('./Configs/Config_Light1');
        initConfig(testConfig);
        Cond1 = global.lights['tl_1'] != null ;
        Cond11 = global.lights['tl_1'].socket == null ;
        Cond2 = global.lights['tl_11'] != null ;
        Cond3 = global.lights['tl_2'] != null ;
        Cond4 = global.lights['tl_22'] != null ;
        Cond44 = global.lights['tl_22sdsd'] == null ;
        Cond5 = trafficGroupsList[0].name == 'C' ;
        Cond6 = trafficGroupsList[1].name == 'B' ;
        Final_Res = Cond1 && Cond11 && Cond2 && Cond3 && Cond4 && Cond5 && Cond6 && Cond44
        console.log(global.lights['tl_1'].durations)
        expect(Final_Res).to.be.true
    });

    it('Test duplicated IDs', () => {
        const configDup = [
          { id: "tl_1", localization_x: 0, localization_y: 0, group: "A" },
          { id: "tl_1", localization_x: 1, localization_y: 1, group: "B" }
        ];
        expect(() => initConfig(configDup)).to.throw(/Duplicate light ID/);
      });
    
    it('testing default durations', () => {
        const testConfig = require('./Configs/Config_Light1');
        initConfig(testConfig);
        //expect(Final_Res).to.be.true  
        expect(global.lights['tl_2'].durations).to.deep.equal(DEFAULT_DURATIONS);
        expect(global.lights['tl_1'].durations).to.deep.equal({ red: 2000, yellow: 2000, green: 4000 });

  
});
});

describe('add new Traffic Light _ Test' , () => {
    it('testing normal behaviour', () => {

        const fakeSocket = {
            end: () => {}
        };
        data_tl_1 = {"command": 60, "lightID": "tl_1"};
        
        const testConfig = require('./Configs/Config_Light1');
        initConfig(testConfig);
        const spy = sinon.spy(global.lights['tl_1'], 'goBlink');
        addNewTrafficLight(data_tl_1, data_tl_1.length, 'newSocket');
        Cond1 = global.lights['tl_1'].socket == 'newSocket' ;
        Final_Res = Cond1;
        expect(Final_Res).to.be.true
        expect(spy.calledOnce).to.be.true;

    });

    it('testing invalid id', () => {
        const fakeSocket = {
            end: () => {}
        };
        data_tl_1 = {"command": 60, "lightID": "ska"};
        let logOutput = '';
        console.log = (msg) => { logOutput = msg; };
        const endSpy = sinon.spy(fakeSocket, 'end');
       
        ret = addNewTrafficLight(data_tl_1, data_tl_1.length, fakeSocket);
        
        // expect(logOutput).to.equal("lightID ist not from Config , ska");
        expect(logOutput).to.include("lightID ist not from Config");
        expect(endSpy.calledOnce).to.be.true;

    });

    it('testing data with no id ', () => {
        const fakeSocket = {
            end: () => {}
        };
        data_tl_1 = {"command": 60, "sthing": "ska"};
        let logOutput = '';
        console.log = (msg) => { logOutput = msg; };
        const originalLog = console.log;
        const endSpy = sinon.spy(fakeSocket, 'end');
        ret = addNewTrafficLight(data_tl_1, data_tl_1.length, fakeSocket);
        
        expect(logOutput).to.include("Missing lightID");
        expect(endSpy.calledOnce).to.be.true;
        // expect(logOutput).to.equal('no Light id in msg');
    });
});

describe('handle TCP received msg _ Test' , () => {
    it('testing normal behaviour', () => {
        const fakeSocket = {
            end: () => {}
        };
        const data_tl_1 = JSON.stringify({ command: 0x60, lightID: 'tl_1' });
        const testConfig = require('./Configs/Config_Light1');
        initConfig(testConfig);
        const endSpy = sinon.spy(fakeSocket, 'end');
        handleReceivedMessage(data_tl_1,fakeSocket);
        Cond1 = global.lights['tl_1'].socket == fakeSocket ;
        console.log('sssssssssss' , global.lights['tl_1'].socket);
        Final_Res = Cond1;
        expect(Final_Res).to.be.true
        expect(endSpy.calledOnce).to.be.false;


    });
    it('testing invalid command', () => {
        const fakeSocket = {
            end: () => {}
        };
        const data_tl_1 = JSON.stringify({ command: 65, lightID: 'tl_1' });
        let logOutput = '';
        console.log = (msg) => { logOutput = msg; };
        const endSpy = sinon.spy(fakeSocket, 'end');
        handleReceivedMessage(data_tl_1,fakeSocket);
        expect(endSpy.calledOnce).to.be.true;
        expect(logOutput).to.include("Invalid Command");
        // expect(logOutput).to.equal('invalid command 65');
    });

    it('testing invalid json', () => {
        const fakeSocket = {
            end: () => {}
        };
        const data_tl_1 = "fake Json";
        let logOutput = '';
        console.log = (msg) => { logOutput = msg; };
        const endSpy = sinon.spy(fakeSocket, 'end');
        handleReceivedMessage(data_tl_1,fakeSocket);
       
       
        expect(logOutput).to.include("JSON error");
        expect(endSpy.calledOnce).to.be.true;
       
        // expect(logOutput).to.equal('invalid command 65');
    });

    
});

describe('TrafficGroup Tests 1', () => {
    let light1, light2, group;

    beforeEach(() => {
        // mocking Traffic lights
        light1 = {
            id: 'tl_1',
            isConnected: () => true,
            goBlink: sinon.spy(),
            goRed: sinon.spy()
        };

        light2 = {
            id: 'tl_2',
            isConnected: () => false,
            goBlink: sinon.spy(),
            goRed: sinon.spy()
        };

        
        global.lights = {
            tl_1: light1,
            tl_2: light2
        };

        group = new TrafficGroup("TestGroup", ["tl_1", "tl_2"]);
    });

    it('testing isReady() returns false if any light is disconnected', () => {
        const result = group.isReady();
        expect(result).to.be.false;
        expect(light1.goBlink.calledOnce).to.be.true;
        expect(light2.goBlink.called).to.be.false;
    });

    it('testing isReady() returns true if all lights are connected', () => {
        light2.isConnected = () => true;
        group = new TrafficGroup("TestGroup", ["tl_1", "tl_2"]);
        const result = group.isReady();
        expect(result).to.be.true;
    });

    it('testing getCurrentLight returns correct light', () => {
        group.currentIndex = 1;
        const current = group.getCurrentLight();
        expect(current.id).to.equal('tl_2');
    });

    it('testing goAllRed only calls goRed on connected lights', async () => {
        light1.isConnected = () => true;
        light2.isConnected = () => false;

        await group.goAllRed();

        expect(light1.goRed.calledOnce).to.be.true;
        expect(light2.goRed.called).to.be.false;
    });
});


describe('TrafficGroup Tests 2', () => {
    let mockLight, group;

    beforeEach(() => {
        mockLight = {
            id: 'tl_1',
            status: TrafficLightStatus.RED,
            durations: { red: 100, yellow: 100, green: 100 },
            isConnected: () => true,
            goRed: sinon.fake.resolves(),
            goGreen: sinon.fake.resolves(),
            goYellow: sinon.fake.resolves()
        };

        global.lights = { tl_1: mockLight };
        group = new TrafficGroup('TestGroup', ['tl_1']);
    });

    const sinon = require('sinon');

    it('getCurrentLight returns correct light by index', () => {
        const light1 = { id: 'tl_1' };
        const light2 = { id: 'tl_2' };
    
        global.lights = { tl_1: light1, tl_2: light2 };
        const group = new TrafficGroup('G', ['tl_1', 'tl_2']);
    
        group.currentIndex = 1;
        expect(group.getCurrentLight().id).to.equal('tl_2');
    
        group.currentIndex = 0;
        expect(group.getCurrentLight().id).to.equal('tl_1');
    });
    it('goAllRed calls goRed only for connected lights', async () => {
        const light1 = {
            id: 'tl_1',
            isConnected: () => true,
            goRed: sinon.fake.resolves()
        };
        const light2 = {
            id: 'tl_2',
            isConnected: () => false,
            goRed: sinon.fake.resolves()
        };
    
        global.lights = { tl_1: light1, tl_2: light2 };
        const group = new TrafficGroup('G', ['tl_1', 'tl_2']);
    
        await group.goAllRed();
    
        expect(light1.goRed.calledOnce).to.be.true;
        expect(light2.goRed.called).to.be.false;
    });
    it('isReady resets notReadyBlinkSent if all lights connected', () => {
        const light1 = {
            id: 'tl_1',
            isConnected: () => true,
            goBlink: sinon.spy()
        };
        const light2 = {
            id: 'tl_2',
            isConnected: () => true,
            goBlink: sinon.spy()
        };
    
        global.lights = { tl_1: light1, tl_2: light2 };
        const group = new TrafficGroup('G', ['tl_1', 'tl_2']);
    
        group.notReadyBlinkSent = true;
        const result = group.isReady();
    
        expect(result).to.be.true;
        expect(group.notReadyBlinkSent).to.be.false;
    });

    it('isReady does not resend goBlink if notReadyBlinkSent is true', () => {
        const light1 = {
            id: 'tl_1',
            isConnected: () => false,
            goBlink: sinon.spy()
        };
    
        global.lights = { tl_1: light1 };
        const group = new TrafficGroup('G', ['tl_1']);
    
        group.notReadyBlinkSent = true;
    
        const result = group.isReady();
    
        expect(result).to.be.false;
        expect(light1.goBlink.called).to.be.false; 
    });

    it('isReady sets reset = true if a light is disconnected', () => {
        const light1 = {
            id: 'tl_1',
            isConnected: () => false,
            goBlink: sinon.spy()
        };
    
        global.lights = { tl_1: light1 };
        const group = new TrafficGroup('G', ['tl_1']);
    
        group.isReady();
    
        expect(group.reset).to.be.true;
    });


    
    
    



});