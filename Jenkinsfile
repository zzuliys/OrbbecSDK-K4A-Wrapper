pipeline {
    agent any

    stages {
        stage('Hello') {
            steps {
                echo 'Hello World'
            }
        }
        stage('pull code') {
            steps {
                // checkout scmGit(branches: [[name: '*/main']], extensions: [], userRemoteConfigs: [[url: 'https://github.com/orbbec/OrbbecSDK-K4A-Wrapper.git']])
                echo 'pull code'
            }
        }
        stage('build code') {
            steps {
                echo 'build code'
            }
        }
    }
}
